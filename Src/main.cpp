/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include <stm32l0xx_hal.h>

/* USER CODE BEGIN Includes */
#include "serial.h"
#include "Conditionals.h"
#include "Configuration.h"
#include "Planner.h"
#include "macros.h"
#include "SREGEmulation.h"
#include "Stopwatch.h"
#include "CommandParser.h"
#include "Stepper.h"
#include "Settings.h"
#include "Temperature.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t commands_in_queue = 0; // Count of commands in the queue
static uint8_t cmd_queue_index_r = 0, // Ring buffer read position
               cmd_queue_index_w = 0; // Ring buffer write position
static char command_queue[BUFSIZE][MAX_CMD_SIZE];

bool Running = true;

/**
 * Cartesian Current Position
 *   Used to track the logical position as moves are queued.
 *   Used by 'line_to_current_position' to do a move after changing it.
 *   Used by 'SYNC_PLAN_POSITION_KINEMATIC' to update 'planner.position'.
 */
float current_position = 0.0;

/**
 * Cartesian Destination
 *   A temporary position, usually applied to 'current_position'.
 *   Set with 'gcode_get_destination' or 'set_destination_to_current'.
 *   'line_to_destination' sets 'current_position' to 'destination'.
 */
float destination = 0.0;

// Initialized by settings.load()
#define AXIS_RELATIVE_MODES false
bool axis_relative_modes = AXIS_RELATIVE_MODES;

// Relative Mode.
static bool relative_mode = false;

float feedrate_mm_s = MMM_TO_MMS(1500.0);

volatile uint32_t counter = 0;

int16_t feedrate_percentage = 100, saved_feedrate_percentage;

// Number of characters read in the current line of serial input
static int serial_count = 0;

/**
 * GCode line number handling. Hosts may opt to include line numbers when
 * sending commands to Marlin, and lines will be checked for sequentiality.
 * M110 N<int> sets the current line number.
 */
static long gcode_N, gcode_LastN;

Stopwatch print_job_timer = Stopwatch();

millis_t previous_cmd_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for syncing.
 */
void sync_plan_position() {
  planner.set_position_mm(current_position);
}
#define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()

/**
 * Set destination and feedrate from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void gcode_get_destination() {
  if (parser.seen('M'))
    destination = parser.value_axis_units() + (axis_relative_modes || relative_mode ? current_position : 0);
  else
    destination = current_position;

  if (parser.linearval('F') > 0.0)
    feedrate_mm_s = MMM_TO_MMS(parser.value_feedrate());
}

  /**
   * Move the planner to the position stored in the destination, which is
   * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
   */
  inline void line_to_destination(const float fr_mm_s) {
    planner.buffer_line(destination, fr_mm_s);
  }
  inline void line_to_destination() { line_to_destination(feedrate_mm_s); }

  /**
   * Prepare a linear move in a Cartesian setup.
   * If Mesh Bed Leveling is enabled, perform a mesh move.
   *
   * Returns true if the caller didn't update current_position.
   */
  inline bool prepare_move_to_destination_cartesian() {
      // Do not use feedrate_percentage for E or Z only moves
      if (current_position == destination)
        line_to_destination();
      else {
        const float fr_scaled = MMS_SCALED(feedrate_mm_s);
            line_to_destination(fr_scaled);
      }
    return false;
  }

  inline void set_current_to_destination() { current_position = destination; }
  inline void set_destination_to_current() { destination = current_position; }

/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 */
void prepare_move_to_destination() {
  refresh_cmd_timeout();

  if (prepare_move_to_destination_cartesian()) return;

  set_current_to_destination();
}

/**************************************************
 ***************** GCode Handlers *****************
 **************************************************/

/**
 * G0, G1: Coordinated movement of X Y Z E axes
 */
inline void gcode_G0_G1(
) {
  if (IsRunning()) {
    gcode_get_destination(); // For X Y Z E F

    prepare_move_to_destination();
  }
}

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
inline void line_to_current_position() {
  planner.buffer_line(current_position, feedrate_mm_s);
}

/**
 *  Plan a move to X and set the current_position
 *  The final current_position may not be the one that was requested
 */
void do_blocking_move_to(const float &lm, const float &fr_mm_s/*=0.0*/) {
  const float old_feedrate_mm_s = feedrate_mm_s;

  feedrate_mm_s = fr_mm_s;
  current_position = lm;
  line_to_current_position();

  stepper.synchronize();

  feedrate_mm_s = old_feedrate_mm_s;
}

/**
 * G92: Set current position to given X
 */
inline void gcode_G92() {
  bool didM = false;

  stepper.synchronize();

  if (parser.seenval('M')) {
      const float v = parser.value_axis_units();

      current_position = v;

      didM = true;
  }
  if (didM)
    SYNC_PLAN_POSITION_KINEMATIC();
}

void lcd_setstatus(const char * const message) {
//  if (lcd_status_message_level > 0) return;
//  strncpy(lcd_status_message, message, 3 * (LCD_WIDTH));
//  lcd_finishstatus(persist);
}

/**
 * M117: Set LCD Status Message
 */
inline void gcode_M117() { lcd_setstatus(parser.string_arg); }

inline void gcode_P0() {
	// TODO: add parser.byte_arg to buffer
}

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void process_next_command() {
  char * const current_command = command_queue[cmd_queue_index_r];

  // Parse the next command in the queue
  parser.parse(current_command);

  // Handle a known M, or P
  switch (parser.command_letter) {
    case 'G': switch (parser.codenum) {

      // G0, G1
      case 0:
      case 1:
        gcode_G0_G1();
        break;

      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;

      case 92: // G92
        gcode_G92();
        break;
    }
    break;

    case 'M': switch (parser.codenum) {
      case 117: // M117: Set LCD message text, if possible
        gcode_M117();
        break;
    }
    break;

    case 'P': switch (parser.codenum) {
      case 0: // P0
    	gcode_P0();
    	break;
    }

//    default: parser.unknown_command_error();
  }

//  ok_to_send();
}

/**
 * Standard idle routine keeps the machine alive
 */
void idle() {
//  host_keepalive();
}

void RCC_ClearFlag(void)
{
  /* Set RMVF bit to clear the reset flags */
  RCC->CSR |= RCC_CSR_RMVF;
}

void setup_powerhold() {
  if (HAS_POWER_SWITCH)
    OUT_WRITE(PS_ON, PS_ON_AWAKE);
}

/**
 * Entry-point: Set up before the program loop
 *  - Set up the  power hold
 *  - Start the serial port
 *  - Print startup messages and diagnostics
 *  - Get EEPROM or default settings
 *  - Initialize managers for:
 *    • temperature
 *    • planner
 *    • watchdog
 *    • stepper
 *    • status LEDs
 */
void setup() {
  setup_powerhold();

  MYSERIAL.begin(BAUDRATE);
//  SERIAL_PROTOCOLLNPGM("start");
//  SERIAL_ECHO_START();
/*
  // Check startup - does nothing if bootloader sets MCUSR to 0
  uint32_t mcu = RCC->CSR;
  if (mcu &  RCC_CSR_PORRSTF) SERIAL_ECHOLNPGM(MSG_POWERUP);
  if (mcu &  RCC_CSR_PINRSTF) SERIAL_ECHOLNPGM(MSG_EXTERNAL_RESET);
  if (mcu &  RCC_CSR_LPWRRSTF) SERIAL_ECHOLNPGM(MSG_BROWNOUT_RESET);
  if (mcu &  RCC_CSR_IWDGRSTF) SERIAL_ECHOLNPGM(MSG_IWATCHDOG_RESET);
  if (mcu &  RCC_CSR_WWDGRSTF) SERIAL_ECHOLNPGM(MSG_WWATCHDOG_RESET);
#if defined(RCC_CSR_FWRSTF)
  if (mcu &  RCC_CSR_FWRSTF) SERIAL_ECHOLNPGM(MSG_MIFAREFIREWALL_RESET);
#endif // RCC_CSR_FWRSTF
  if (mcu & RCC_CSR_SFTRSTF) SERIAL_ECHOLNPGM(MSG_SOFTWARE_RESET);
  if (mcu & RCC_CSR_OBLRSTF) SERIAL_ECHOLNPGM(MSG_OBL_RESET);
*/
  RCC_ClearFlag();

  // Load data from EEPROM if available (or use defaults)
  // This also updates variables in the planner, elsewhere
  (void)settings.load();

  current_position = 0;

  // Vital to init stepper/planner equivalent for current_position
  SYNC_PLAN_POSITION_KINEMATIC();

  thermalManager.init();    // Initialize temperature loop

  #if ENABLED(USE_WATCHDOG)
    watchdog_init();
  #endif

  stepper.init();    // Initialize stepper, this enables interrupts!

  #if ENABLED(RGB_LED) || ENABLED(RGBW_LED)
    SET_OUTPUT(RGB_LED_R_PIN);
    SET_OUTPUT(RGB_LED_G_PIN);
    SET_OUTPUT(RGB_LED_B_PIN);
    #if ENABLED(RGBW_LED)
      SET_OUTPUT(RGB_LED_W_PIN);
    #endif
  #endif
}

/**
 * Send a "Resend: nnn" message to the host to
 * indicate that a command needs to be re-sent.
 */
void FlushSerialRequestResend() {
  MYSERIAL.flush();
}

void gcode_line_error(bool doFlush = true) {
  if (doFlush) FlushSerialRequestResend();
  serial_count = 0;
}

/**
 * Once a new command is in the ring buffer, call this to commit it
 */
inline void _commit_command(bool say_ok) {
  if (++cmd_queue_index_w >= BUFSIZE) cmd_queue_index_w = 0;
  commands_in_queue++;
}

/**
 * Copy a command from RAM into the main command buffer.
 * Return true if the command was successfully added.
 * Return false for a full buffer, or if the 'command' is a comment.
 */
inline bool _enqueuecommand(const char* cmd, bool say_ok=false) {
  if (*cmd == ';' || commands_in_queue >= BUFSIZE) return false;
  strcpy(command_queue[cmd_queue_index_w], cmd);
  _commit_command(say_ok);
  return true;
}

/**
 * Get all commands waiting on the serial port and queue them.
 * Exit when the buffer is full or when no more characters are
 * left on the serial port.
 */
inline void get_serial_commands() {
  static char serial_line_buffer[MAX_CMD_SIZE];
  static bool serial_comment_mode = false;

  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  while (commands_in_queue < BUFSIZE && MYSERIAL.available() > 0) {

    char serial_char = MYSERIAL.read();

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      serial_comment_mode = false; // end of line == end of comment

      if (!serial_count) continue; // skip empty lines

      serial_line_buffer[serial_count] = 0; // terminate string
      serial_count = 0; //reset buffer

      char* command = serial_line_buffer;

      while (*command == ' ') command++; // skip any leading spaces
      char *npos = (*command == 'N') ? command : NULL, // Require the N parameter to start the line
           *apos = strchr(command, '*');

      if (npos) {

        gcode_N = strtol(npos + 1, NULL, 10);

        if (gcode_N != gcode_LastN + 1) {
          gcode_line_error();
          return;
        }

        gcode_LastN = gcode_N;
        // if no errors, continue parsing
      }
      else if (apos) { // No '*' without 'N'
        gcode_line_error(false);
        return;
      }

      // Movement commands alert when stopped
      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          const int codenum = strtol(gpos + 1, NULL, 10);
          switch (codenum) {
            case 0:
            case 1:
            case 2:
            case 3:
              break;
          }
        }
      }

      // Add the command to the queue
      _enqueuecommand(serial_line_buffer, true);
    }
    else if (serial_count >= MAX_CMD_SIZE - 1) {
      // Keep fetching, but ignore normal characters beyond the max length
      // The command will be injected when EOL is reached
    }
    else if (serial_char == '\\') {  // Handle escapes
      if (MYSERIAL.available() > 0) {
        // if we have one more character, copy it over
        serial_char = MYSERIAL.read();
        if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
      }
      // otherwise do nothing
    }
    else { // it's not a newline, carriage return or escape char
      if (serial_char == ';') serial_comment_mode = true;
      if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
    }

  } // queue has space, serial has data
}

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (injected_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void get_available_commands() {
  get_serial_commands();
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  SysTick_Config(SystemCoreClock / 16000);
  HAL_NVIC_EnableIRQ(SysTick_IRQn);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

  /* USER CODE BEGIN 2 */
  setup();
  /* Enable lpusart 1 receive IRQ */
  for (;;) {
	  if (commands_in_queue < BUFSIZE) get_available_commands();

	  if (commands_in_queue) {
		  process_next_command();

		  // The queue may be reset by a command handler or by code invoked by idle() within a handler
		  if (commands_in_queue) {
		      --commands_in_queue;
		      if (++cmd_queue_index_r >= BUFSIZE) cmd_queue_index_r = 0;
		  }
	  }

	  idle();

      /* Main cycle */
//      while (rx_i != rx_e) {
//          /* echo here */
//          transmit(&huart2, rx_buf[RXBUF_MSK & rx_e]);
//          rx_e++;
//      }
//      while (rx_i != rx_o) {
//          /* parse here */
//          /* ... */
//          rx_o++;
//      }
      /* Power save
      while (tx_busy);
      HAL_UART_DeInit(&huart1);
      */
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void SysTick_Handler(void) {
  counter++;
}

void suicide() {
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, LOW);
  #endif
}

/**
 * Kill all activity and lock the machine.
 * After this the machine will need to be reset.
 */
void kill() {
//  SERIAL_ERROR_START();
//  SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

  thermalManager.disable_all_heaters();
  disable_all_steppers();

  HAL_Delay(600); // Wait a short time (allows messages to get out before shutting down.
  cli(); // Stop interrupts

  HAL_Delay(250); //Wait to ensure all interrupts routines stopped
  thermalManager.disable_all_heaters(); //turn off heaters again

  if (HAS_POWER_SWITCH)
    SET_INPUT(PS_ON);

  suicide();
  while (1) {
    #if ENABLED(USE_WATCHDOG)
      watchdog_reset();
    #endif
  } // Wait for reset
}

void disable_all_steppers() {
  disable_MOTOR();
}

void setInput(GPIO_TypeDef  *port, uint32_t pin, uint32_t mode) {
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = mode;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}  // set to input, which allows it to be pulled high by pullups

void setOutput(GPIO_TypeDef  *port, uint32_t pin) {
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}  // set to output

uint32_t millis() {
  return counter;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(const char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
