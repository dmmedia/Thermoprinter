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
#include "main.h"
#include "macros.h"
#include "serial.h"
#include "Conditionals.h"
#include "Configuration.h"
#include "Planner.h"
#include "SREGEmulation.h"
#include "Stopwatch.h"
#include "CommandParser.h"
#include "Stepper.h"
#include "Settings.h"
#include "Temperature.h"
#include "gpio.h"
#include "rcc.h"

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
long int current_position = 0;

/**
 * Cartesian Destination
 *   A temporary position, usually applied to 'current_position'.
 *   Set with 'gcode_get_destination' or 'set_destination_to_current'.
 *   'line_to_destination' sets 'current_position' to 'destination'.
 */
long int destination = 0;

// Initialized by settings.load()
#define AXIS_RELATIVE_MODES false
bool axis_relative_modes = AXIS_RELATIVE_MODES;

// Relative Mode.
static bool relative_mode = false;

float feedrate_mm_s = MMM_TO_MMS(1500.0);

int16_t feedrate_percentage = 100, saved_feedrate_percentage;

// Number of characters read in the current line of serial input
static int serial_count = 0;

Stopwatch print_job_timer = Stopwatch();

millis_t previous_cmd_ms = 0;

static bool send_ok[BUFSIZE];

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
 *  - Set destination from command line
 *  - Set to current for non-movement commands
 */
void gcode_get_destination() {
	if (parser.command_letter == 'M') {
	    destination = parser.value_axis_units() + current_position;
	} else if (parser.command_letter == 'P') {
		destination = 2 + current_position;
	} else {
		destination = current_position;
	}
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
 * M0: Stepped movement of motor
 */
inline void gcode_M0(
) {
  if (IsRunning()) {
    gcode_get_destination();

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
 * G92: Set current position to 0
 */
inline void gcode_G92() {
  stepper.synchronize();

  current_position = 0;

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

  SERIAL_ECHO_START();
  SERIAL_ECHOLN(current_command);

  // Parse the next command in the queue
  parser.parse(current_command);

  // Handle a known M, or P
  switch (parser.command_letter) {
    case 'M': switch (parser.codenum) {
      case 0:
        gcode_M0();
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

    case 'P': switch (parser.codenum) {
      case 0: // P0
    	gcode_P0();
    	break;
    }
    break;

    default:
      parser.unknown_command_error();
  }

  ok_to_send();
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
//  if (HAS_POWER_SWITCH)
//    OUT_WRITE(PS_ON, PS_ON_AWAKE);
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
  SERIAL_PROTOCOLLNPGM("start");

  SERIAL_ECHOPGM(MSG_THERMOPRINTER);
  SERIAL_CHAR(' ');
  SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
  SERIAL_EOL();

  RCC_ClearFlag();

  // Send "ok" after commands by default
  for (int8_t i = 0; i < BUFSIZE; i++) send_ok[i] = true;

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
  send_ok[cmd_queue_index_w] = say_ok;
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

  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  while (commands_in_queue < BUFSIZE && MYSERIAL.available() > 0) {

    char serial_char = MYSERIAL.read();

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      if (!serial_count) continue; // skip empty lines

      serial_line_buffer[serial_count] = 0; // terminate string
      serial_count = 0; //reset buffer

      char* command = serial_line_buffer;

      while (*command == ' ') command++; // skip any leading spaces

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
        serial_line_buffer[serial_count++] = serial_char;
      }
      // otherwise do nothing
    }
    else { // it's not a newline, carriage return or escape char
      serial_line_buffer[serial_count++] = serial_char;
    }

  } // queue has space, serial has data
}

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (injected_commands_P)
 *  - The active serial input (usually USB)
 */
void get_available_commands() {
  get_serial_commands();
}

/**
  * @brief  Enable the FLASH preread buffer.
  * @retval none
  */
#define FLASH_PREREAD_BUFFER_ENABLE()       SET_BIT((FLASH->ACR), FLASH_ACR_PRE_READ)

/**
  * Initializes the Global MSP.
  */
void MspInit(void)
{
  RCC_SYSCFG_CLK_ENABLE();
  RCC_PWR_CLK_ENABLE();

  /* System interrupt init*/
  /* SVC_IRQn interrupt configuration */
  NVIC_SetPriority(SVC_IRQn, 0);
  /* PendSV_IRQn interrupt configuration */
  NVIC_SetPriority(PendSV_IRQn, 0);
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 0);
}

/**
  * @brief This function configures the Flash prefetch, Flash preread and Buffer cache,
  *        Configures time base source, NVIC and Low level hardware
  * @note This function is called at the beginning of program after reset and before
  *       the clock configuration
  * @note The time base configuration is based on MSI clock when exiting from Reset.
  *       Once done, time base tick start incrementing.
  *        In the default implementation,Systick is used as source of time base.
  *        the tick variable is incremented each 1ms in its ISR.
  * @retval HAL status
  */
StatusTypeDef Init(void)
{
  /* Configure Buffer cache, Flash prefetch,  Flash preread */
  FLASH_PREREAD_BUFFER_ENABLE();

  /* Use systick as time base source and configure 1ms tick (default clock after Reset is MSI) */
  InitTick(TICK_INT_PRIORITY);

  /* Init the low level hardware */
  MspInit();

  /* Return function status */
  return STATUS_OK;
}

int main(void)
{
  Init();
  SystemClock_Config();
  SysTick_Config(SystemCoreClock / 16000);
  NVIC_EnableIRQ(SysTick_IRQn);
  MX_GPIO_Init();
  setup();
  while (true) {
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
  }
}

/** @brief  macros configure the main internal regulator output voltage.
  *         When exiting Low Power Run Mode or during dynamic voltage scaling configuration,
  *         the reference manual recommends to poll PWR_FLAG_REGLP bit to wait for the regulator
  *         to reach main mode (resp. to get stabilized) for a transition from 0 to 1.
  *         Only then the clock can be increased.
  *
  * @param  __REGULATOR__: specifies the regulator output voltage to achieve
  *         a tradeoff between performance and power consumption when the device does
  *         not operate at the maximum frequency (refer to the datasheets for more details).
  *          This parameter can be one of the following values:
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE1: Regulator voltage output Scale 1 mode,
  *                                                System frequency up to 32 MHz.
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE2: Regulator voltage output Scale 2 mode,
  *                                                System frequency up to 16 MHz.
  *            @arg PWR_REGULATOR_VOLTAGE_SCALE3: Regulator voltage output Scale 3 mode,
  *                                                System frequency up to 4.2 MHz
  * @retval None
  */
#define PWR_VOLTAGESCALING_CONFIG(__REGULATOR__) (MODIFY_REG(PWR->CR, PWR_CR_VOS, (__REGULATOR__)))

#define RCC_CLOCKTYPE_SYSCLK             ((uint32_t)0x00000001) /*!< SYSCLK to configure */
#define RCC_CLOCKTYPE_HCLK               ((uint32_t)0x00000002) /*!< HCLK to configure */
#define RCC_CLOCKTYPE_PCLK1              ((uint32_t)0x00000004) /*!< PCLK1 to configure */
#define RCC_CLOCKTYPE_PCLK2              ((uint32_t)0x00000008) /*!< PCLK2 to configure */

#define RCC_SYSCLKSOURCE_MSI             RCC_CFGR_SW_MSI /*!< MSI selected as system clock */
#define RCC_SYSCLKSOURCE_HSI             RCC_CFGR_SW_HSI /*!< HSI selected as system clock */
#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE /*!< HSE selected as system clock */
#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL /*!< PLL selected as system clock */

/** @defgroup RCC_APB1_APB2_Clock_Source APB1 APB2 Clock Source
  * @{
  */
#define RCC_HCLK_DIV1                    RCC_CFGR_PPRE1_DIV1  /*!< HCLK not divided */
#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2  /*!< HCLK divided by 2 */
#define RCC_HCLK_DIV4                    RCC_CFGR_PPRE1_DIV4  /*!< HCLK divided by 4 */
#define RCC_HCLK_DIV8                    RCC_CFGR_PPRE1_DIV8  /*!< HCLK divided by 8 */
#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE1_DIV16 /*!< HCLK divided by 16 */

/**
  * @brief  Macro to configure the system clock source.
  * @param  __SYSCLKSOURCE__ specifies the system clock source.
  *          This parameter can be one of the following values:
  *              @arg @ref RCC_SYSCLKSOURCE_MSI MSI oscillator is used as system clock source.
  *              @arg @ref RCC_SYSCLKSOURCE_HSI HSI oscillator is used as system clock source.
  *              @arg @ref RCC_SYSCLKSOURCE_HSE HSE oscillator is used as system clock source.
  *              @arg @ref RCC_SYSCLKSOURCE_PLLCLK PLL output is used as system clock source.
  */
#define RCC_SYSCLK_CONFIG(__SYSCLKSOURCE__) \
                  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, (__SYSCLKSOURCE__))

/**
  * @brief  Initializes the CPU, AHB and APB buses clocks according to the specified
  *         parameters in the RCC_ClkInitStruct.
  * @param  RCC_ClkInitStruct pointer to an RCC_OscInitTypeDef structure that
  *         contains the configuration information for the RCC peripheral.
  * @param  FLatency FLASH Latency
  *          The value of this parameter depend on device used within the same series
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  *         and updated by @ref HAL_RCC_GetHCLKFreq() function called within this function
  *
  * @note   The MSI is used (enabled by hardware) as system clock source after
  *         start-up from Reset, wake-up from STOP and STANDBY mode, or in case
  *         of failure of the HSE used directly or indirectly as system clock
  *         (if the Clock Security System CSS is enabled).
  *
  * @note   A switch from one clock source to another occurs only if the target
  *         clock source is ready (clock stable after start-up delay or PLL locked).
  *         If a clock source which is not yet ready is selected, the switch will
  *         occur when the clock source will be ready.
  *         You can use @ref HAL_RCC_GetClockConfig() function to know which clock is
  *         currently used as system clock source.
  * @note   Depending on the device voltage range, the software has to set correctly
  *         HPRE[3:0] bits to ensure that HCLK not exceed the maximum allowed frequency
  *         (for more details refer to section above "Initialization/de-initialization functions")
  * @retval HAL status
  */
StatusTypeDef RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency)
{
  uint32_t tickstart = 0U;

  /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
  must be correctly programmed according to the frequency of the CPU clock
  (HCLK) and the supply voltage of the device. */

  /* Increasing the number of wait states because of higher CPU frequency */
  if(FLatency > (FLASH->ACR & FLASH_ACR_LATENCY))
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
    if((FLASH->ACR & FLASH_ACR_LATENCY) != FLatency)
    {
      return STATUS_ERROR;
    }
  }

  /*-------------------------- HCLK Configuration --------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK)
  {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);
  }

  /*------------------------- SYSCLK Configuration ---------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK)
  {
    /* HSE is selected as System Clock Source */
    if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
    {
      /* Check the HSE ready flag */
      if(RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
      {
        return STATUS_ERROR;
      }
    }
    /* PLL is selected as System Clock Source */
    else if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
    {
      /* Check the PLL ready flag */
      if(RCC_GET_FLAG(RCC_FLAG_PLLRDY) == RESET)
      {
        return STATUS_ERROR;
      }
    }
    /* HSI is selected as System Clock Source */
    else if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSI)
    {
      /* Check the HSI ready flag */
      if(RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
      {
        return STATUS_ERROR;
      }
    }
    /* MSI is selected as System Clock Source */
    else
    {
      /* Check the MSI ready flag */
      if(RCC_GET_FLAG(RCC_FLAG_MSIRDY) == RESET)
      {
        return STATUS_ERROR;
      }
    }
    RCC_SYSCLK_CONFIG(RCC_ClkInitStruct->SYSCLKSource);

    /* Get Start Tick */
    tickstart = GetTick();

    if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE)
    {
      while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSE)
      {
        if((GetTick() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE)
        {
          return STATUS_TIMEOUT;
        }
      }
    }
    else if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK)
    {
      while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
      {
        if((GetTick() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE)
        {
          return STATUS_TIMEOUT;
        }
      }
    }
    else if(RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSI)
    {
      while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSI)
      {
        if((GetTick() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE)
        {
          return STATUS_TIMEOUT;
        }
      }
    }
    else
    {
      while(RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_MSI)
      {
        if((GetTick() - tickstart ) > CLOCKSWITCH_TIMEOUT_VALUE)
        {
          return STATUS_TIMEOUT;
        }
      }
    }
  }
  /* Decreasing the number of wait states because of lower CPU frequency */
  if(FLatency < (FLASH->ACR & FLASH_ACR_LATENCY))
  {
    /* Program the new number of wait states to the LATENCY bits in the FLASH_ACR register */
    FLASH_SET_LATENCY(FLatency);

    /* Check that the new number of wait states is taken into account to access the Flash
    memory by reading the FLASH_ACR register */
    if((FLASH->ACR & FLASH_ACR_LATENCY) != FLatency)
    {
      return STATUS_ERROR;
    }
  }

  /*-------------------------- PCLK1 Configuration ---------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1)
  {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);
  }

  /*-------------------------- PCLK2 Configuration ---------------------------*/
  if(((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2)
  {
    MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3));
  }

  /* Update the SystemCoreClock global variable */
  SystemCoreClock = RCC_GetSysClockFreq() >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE)>> RCC_CFGR_HPRE_BITNUMBER];

  /* Configure the source of time base considering new system clocks settings*/
  InitTick (TICK_INT_PRIORITY);

  return STATUS_OK;
}

/** @brief  Macro to force the Backup domain reset.
  * @note   This function resets the RTC peripheral (including the backup registers)
  *         and the RTC clock source selection in RCC_CSR register.
  * @note   The BKPSRAM is not affected by this reset.
  */
#define RCC_BACKUPRESET_FORCE()  SET_BIT(RCC->CSR, RCC_CSR_RTCRST)

/** @brief  Macros to release the Backup domain reset.
  */
#define RCC_BACKUPRESET_RELEASE() CLEAR_BIT(RCC->CSR, RCC_CSR_RTCRST)

/** @brief Macro to configure the RTC clock (RTCCLK).
  * @note   As the RTC clock configuration bits are in the Backup domain and write
  *         access is denied to this domain after reset, you have to enable write
  *         access using the Power Backup Access macro before to configure
  *         the RTC clock source (to be done once after reset).
  * @note   Once the RTC clock is configured it cannot be changed unless the
  *         Backup domain is reset using @ref __HAL_RCC_BACKUPRESET_FORCE() macro, or by
  *         a Power On Reset (POR).
  * @note   RTC prescaler cannot be modified if HSE is enabled (HSEON = 1).
  *
  * @param  __RTC_CLKSOURCE__ specifies the RTC clock source.
  *          This parameter can be one of the following values:
  *             @arg @ref RCC_RTCCLKSOURCE_NO_CLK No clock selected as RTC clock
  *             @arg @ref RCC_RTCCLKSOURCE_LSE LSE selected as RTC clock
  *             @arg @ref RCC_RTCCLKSOURCE_LSI LSI selected as RTC clock
  *             @arg @ref RCC_RTCCLKSOURCE_HSE_DIV2 HSE divided by 2 selected as RTC clock
  *             @arg @ref RCC_RTCCLKSOURCE_HSE_DIV4 HSE divided by 4 selected as RTC clock
  *             @arg @ref RCC_RTCCLKSOURCE_HSE_DIV8 HSE divided by 8 selected as RTC clock
  *             @arg @ref RCC_RTCCLKSOURCE_HSE_DIV16 HSE divided by 16 selected as RTC clock
  * @note   If the LSE or LSI is used as RTC clock source, the RTC continues to
  *         work in STOP and STANDBY modes, and can be used as wakeup source.
  *         However, when the HSE clock is used as RTC clock source, the RTC
  *         cannot be used in STOP and STANDBY modes.
  * @note   The maximum input clock frequency for RTC is 1MHz (when using HSE as
  *         RTC clock source).
  */
#define RCC_RTC_CLKPRESCALER(__RTC_CLKSOURCE__) do { \
            if(((__RTC_CLKSOURCE__) & RCC_CSR_RTCSEL_HSE) == RCC_CSR_RTCSEL_HSE)          \
            {                                                                             \
              MODIFY_REG(RCC->CR, RCC_CR_RTCPRE, ((__RTC_CLKSOURCE__) & RCC_CR_RTCPRE));  \
            }                                                                             \
          } while (0)

#define RCC_RTC_CONFIG(__RTC_CLKSOURCE__) do { \
                                      RCC_RTC_CLKPRESCALER(__RTC_CLKSOURCE__);      \
                                      RCC->CSR |= ((__RTC_CLKSOURCE__) & RCC_CSR_RTCSEL); \
                                    } while (0)

#if defined (RCC_CCIPR_USART1SEL)
/** @brief Macro to configure the USART1 clock (USART1CLK).
  *
  * @param  __USART1_CLKSOURCE__ specifies the USART1 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_USART1CLKSOURCE_PCLK2 PCLK2 selected as USART1 clock
  *            @arg @ref RCC_USART1CLKSOURCE_HSI HSI selected as USART1 clock
  *            @arg @ref RCC_USART1CLKSOURCE_SYSCLK System Clock selected as USART1 clock
  *            @arg @ref RCC_USART1CLKSOURCE_LSE LSE selected as USART1 clock
  */
#define RCC_USART1_CONFIG(__USART1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_USART1SEL, (uint32_t)(__USART1_CLKSOURCE__))

/** @brief  Macro to get the USART1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_USART1CLKSOURCE_PCLK2 PCLK2 selected as USART1 clock
  *            @arg @ref RCC_USART1CLKSOURCE_HSI HSI selected as USART1 clock
  *            @arg @ref RCC_USART1CLKSOURCE_SYSCLK System Clock selected as USART1 clock
  *            @arg @ref RCC_USART1CLKSOURCE_LSE LSE selected as USART1 clock
  */
#define RCC_GET_USART1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_USART1SEL)))
#endif /* RCC_CCIPR_USART1SEL */

/** @brief Macro to configure the USART2 clock (USART2CLK).
  *
  * @param  __USART2_CLKSOURCE__ specifies the USART2 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_USART2CLKSOURCE_PCLK1 PCLK1 selected as USART2 clock
  *            @arg @ref RCC_USART2CLKSOURCE_HSI HSI selected as USART2 clock
  *            @arg @ref RCC_USART2CLKSOURCE_SYSCLK System Clock selected as USART2 clock
  *            @arg @ref RCC_USART2CLKSOURCE_LSE LSE selected as USART2 clock
  */
#define RCC_USART2_CONFIG(__USART2_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_USART2SEL, (uint32_t)(__USART2_CLKSOURCE__))

/** @brief  Macro to get the USART2 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_USART2CLKSOURCE_PCLK1 PCLK1 selected as USART2 clock
  *            @arg @ref RCC_USART2CLKSOURCE_HSI HSI selected as USART2 clock
  *            @arg @ref RCC_USART2CLKSOURCE_SYSCLK System Clock selected as USART2 clock
  *            @arg @ref RCC_USART2CLKSOURCE_LSE LSE selected as USART2 clock
  */
#define RCC_GET_USART2_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_USART2SEL)))

/** @brief Macro to configure the LPUART1 clock (LPUART1CLK).
  *
  * @param  __LPUART1_CLKSOURCE__ specifies the LPUART1 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_LPUART1CLKSOURCE_PCLK1 PCLK1 selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_HSI HSI selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_SYSCLK System Clock selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_LSE LSE selected as LPUART1 clock
  */
#define RCC_LPUART1_CONFIG(__LPUART1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPUART1SEL, (uint32_t)(__LPUART1_CLKSOURCE__))

/** @brief  Macro to get the LPUART1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_LPUART1CLKSOURCE_PCLK1 PCLK1 selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_HSI HSI selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_SYSCLK System Clock selected as LPUART1 clock
  *            @arg @ref RCC_LPUART1CLKSOURCE_LSE LSE selected as LPUART1 clock
  */
#define RCC_GET_LPUART1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_LPUART1SEL)))

/** @brief Macro to configure the I2C1 clock (I2C1CLK).
  *
  * @param  __I2C1_CLKSOURCE__ specifies the I2C1 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_I2C1CLKSOURCE_PCLK1 PCLK1 selected as I2C1 clock
  *            @arg @ref RCC_I2C1CLKSOURCE_HSI HSI selected as I2C1 clock
  *            @arg @ref RCC_I2C1CLKSOURCE_SYSCLK System Clock selected as I2C1 clock
  */
#define RCC_I2C1_CONFIG(__I2C1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_I2C1SEL, (uint32_t)(__I2C1_CLKSOURCE__))

/** @brief  Macro to get the I2C1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_I2C1CLKSOURCE_PCLK1 PCLK1 selected as I2C1 clock
  *            @arg @ref RCC_I2C1CLKSOURCE_HSI HSI selected as I2C1 clock
  *            @arg @ref RCC_I2C1CLKSOURCE_SYSCLK System Clock selected as I2C1 clock
  */
#define RCC_GET_I2C1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_I2C1SEL)))

/** @brief Macro to configure the LPTIM1 clock (LPTIM1CLK).
  *
  * @param  __LPTIM1_CLKSOURCE__ specifies the LPTIM1 clock source.
  *          This parameter can be one of the following values:
  *            @arg @ref RCC_LPTIM1CLKSOURCE_PCLK PCLK selected as LPTIM1 clock
  *            @arg @ref RCC_LPTIM1CLKSOURCE_LSI  HSI  selected as LPTIM1 clock
  *            @arg @ref RCC_LPTIM1CLKSOURCE_HSI  LSI  selected as LPTIM1 clock
  *            @arg @ref RCC_LPTIM1CLKSOURCE_LSE  LSE  selected as LPTIM1 clock
  */
#define RCC_LPTIM1_CONFIG(__LPTIM1_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIM1SEL, (uint32_t)(__LPTIM1_CLKSOURCE__))

/** @brief  Macro to get the LPTIM1 clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_LPTIM1CLKSOURCE_PCLK PCLK selected as LPUART1 clock
  *            @arg @ref RCC_LPTIM1CLKSOURCE_LSI  HSI selected as LPUART1 clock
  *            @arg @ref RCC_LPTIM1CLKSOURCE_HSI  System Clock selected as LPUART1 clock
  *            @arg @ref RCC_LPTIM1CLKSOURCE_LSE  LSE selected as LPUART1 clock
  */
#define RCC_GET_LPTIM1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_LPTIM1SEL)))

#if defined(USB)
/** @brief  Macro to configure the USB clock (USBCLK).
  * @param  __USB_CLKSOURCE__ specifies the USB clock source.
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_USBCLKSOURCE_HSI48  HSI48 selected as USB clock
  *            @arg @ref RCC_USBCLKSOURCE_PLL PLL Clock selected as USB clock
  */
#define RCC_USB_CONFIG(__USB_CLKSOURCE__) \
                  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_HSI48SEL, (uint32_t)(__USB_CLKSOURCE__))

/** @brief  Macro to get the USB clock source.
  * @retval The clock source can be one of the following values:
  *            @arg @ref RCC_USBCLKSOURCE_HSI48  HSI48 selected as USB clock
  *            @arg @ref RCC_USBCLKSOURCE_PLL PLL Clock selected as USB clock
  */
#define RCC_GET_USB_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_HSI48SEL)))
#endif /* USB */

/**
  * @brief  Initializes the RCC extended peripherals clocks according to the specified
  *         parameters in the RCC_PeriphCLKInitTypeDef.
  * @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
  *         contains the configuration information for the Extended Peripherals clocks(USART1,USART2, LPUART1,
  *         I2C1, I2C3, RTC, USB/RNG  and LPTIM1 clocks).
  * @retval status
  * @note   If STATUS_ERROR returned, first switch-OFF HSE clock oscillator with @ref RCC_OscConfig()
  *         to possibly update HSE divider.
  */
StatusTypeDef RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit)
{
  uint32_t tickstart = 0U;
  uint32_t temp_reg = 0U;

  /*------------------------------- RTC/LCD Configuration ------------------------*/
  if ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC)
#if defined(LCD)
   || (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LCD) == RCC_PERIPHCLK_LCD)
#endif /* LCD */
     )
  {
    FlagStatus       pwrclkchanged = RESET;

    /* As soon as function is called to change RTC clock source, activation of the
       power domain is done. */
    /* Requires to enable write access to Backup Domain of necessary */
    if(RCC_PWR_IS_CLK_DISABLED())
    {
      RCC_PWR_CLK_ENABLE();
      pwrclkchanged = SET;
    }

    if(IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
    {
      /* Enable write access to Backup domain */
      SET_BIT(PWR->CR, PWR_CR_DBP);

      /* Wait for Backup domain Write protection disable */
      tickstart = GetTick();

      while(IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
      {
        if((GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
        {
          return STATUS_TIMEOUT;
        }
      }
    }

    /* Check if user wants to change HSE RTC prescaler whereas HSE is enabled */
    temp_reg = (RCC->CR & RCC_CR_RTCPRE);
    if ((temp_reg != (PeriphClkInit->RTCClockSelection & RCC_CR_RTCPRE))
#if defined (LCD)
     || (temp_reg != (PeriphClkInit->LCDClockSelection & RCC_CR_RTCPRE))
#endif /* LCD */
       )
    { /* Check HSE State */
      if (((PeriphClkInit->RTCClockSelection & RCC_CSR_RTCSEL) == RCC_CSR_RTCSEL_HSE) && IS_BIT_SET(RCC->CR, RCC_CR_HSERDY))
      {
        /* To update HSE divider, first switch-OFF HSE clock oscillator*/
        return STATUS_ERROR;
      }
    }

    /* Reset the Backup domain only if the RTC Clock source selection is modified from reset value */
    temp_reg = (RCC->CSR & RCC_CSR_RTCSEL);

    if((temp_reg != 0x00000000U) && (((temp_reg != (PeriphClkInit->RTCClockSelection & RCC_CSR_RTCSEL)) \
      && (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC))
#if defined(LCD)
      || ((temp_reg != (PeriphClkInit->LCDClockSelection & RCC_CSR_RTCSEL)) \
       && (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LCD) == RCC_PERIPHCLK_LCD))
#endif /* LCD */
     ))
    {
      /* Store the content of CSR register before the reset of Backup Domain */
      temp_reg = (RCC->CSR & ~(RCC_CSR_RTCSEL));

      /* RTC Clock selection can be changed only if the Backup Domain is reset */
      RCC_BACKUPRESET_FORCE();
      RCC_BACKUPRESET_RELEASE();

      /* Restore the Content of CSR register */
      RCC->CSR = temp_reg;

       /* Wait for LSERDY if LSE was enabled */
      if (IS_BIT_SET(temp_reg, RCC_CSR_LSEON))
      {
        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till LSE is ready */
        while(RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
        {
          if((GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }
      }
    }
    RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);

    /* Require to disable power clock if necessary */
    if(pwrclkchanged == SET)
    {
      RCC_PWR_CLK_DISABLE();
    }
  }

#if defined (RCC_CCIPR_USART1SEL)
  /*------------------------------- USART1 Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART1) == RCC_PERIPHCLK_USART1)
  {
    /* Configure the USART1 clock source */
    RCC_USART1_CONFIG(PeriphClkInit->Usart1ClockSelection);
  }
#endif /* RCC_CCIPR_USART1SEL */

  /*----------------------------- USART2 Configuration --------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART2) == RCC_PERIPHCLK_USART2)
  {
    /* Configure the USART2 clock source */
    RCC_USART2_CONFIG(PeriphClkInit->Usart2ClockSelection);
  }

  /*------------------------------ LPUART1 Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPUART1) == RCC_PERIPHCLK_LPUART1)
  {
    /* Configure the LPUAR1 clock source */
    RCC_LPUART1_CONFIG(PeriphClkInit->Lpuart1ClockSelection);
  }

  /*------------------------------ I2C1 Configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C1) == RCC_PERIPHCLK_I2C1)
  {
    /* Configure the I2C1 clock source */
    RCC_I2C1_CONFIG(PeriphClkInit->I2c1ClockSelection);
  }

#if defined(USB)
 /*---------------------------- USB and RNG configuration --------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USB) == (RCC_PERIPHCLK_USB))
  {
    RCC_USB_CONFIG(PeriphClkInit->UsbClockSelection);
  }
#endif /* USB */

  /*---------------------------- LPTIM1 configuration ------------------------*/
  if(((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPTIM1) == (RCC_PERIPHCLK_LPTIM1))
  {
    RCC_LPTIM1_CONFIG(PeriphClkInit->LptimClockSelection);
  }

  return STATUS_OK;
}

  /** @defgroup CORTEX_SysTick_clock_source CORTEX SysTick Clock Source
    * @{
    */
  #define SYSTICK_CLKSOURCE_HCLK_DIV8    ((uint32_t)0x00000000U)
  #define SYSTICK_CLKSOURCE_HCLK         ((uint32_t)0x00000004U)
  #define IS_SYSTICK_CLK_SOURCE(__SOURCE__) (((__SOURCE__) == SYSTICK_CLKSOURCE_HCLK) || \
                                         ((__SOURCE__) == SYSTICK_CLKSOURCE_HCLK_DIV8))

  /**
    * @brief  Configures the SysTick clock source.
    * @param  CLKSource: specifies the SysTick clock source.
    *          This parameter can be one of the following values:
    *             @arg SYSTICK_CLKSOURCE_HCLK_DIV8: AHB clock divided by 8 selected as SysTick clock source.
    *             @arg SYSTICK_CLKSOURCE_HCLK: AHB clock selected as SysTick clock source.
    * @retval None
    */
  void SYSTICK_CLKSourceConfig(uint32_t CLKSource)
  {
    if (CLKSource == SYSTICK_CLKSOURCE_HCLK)
    {
      SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
    }
    else
    {
      SysTick->CTRL &= ~SYSTICK_CLKSOURCE_HCLK;
    }
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
  PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (RCC_OscConfig(&RCC_OscInitStruct) != STATUS_OK)
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

  if (RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != STATUS_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_LPUART1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  if (RCCEx_PeriphCLKConfig(&PeriphClkInit) != STATUS_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  SysTick_Config(SystemCoreClock/1000);

    /**Configure the Systick 
    */
  SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 0);
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
  RCC_GPIOA_CLK_ENABLE();
  RCC_GPIOB_CLK_ENABLE();
  RCC_GPIOC_CLK_ENABLE();
}

/* USER CODE BEGIN 4 */
void SysTick_Handler(void) {
  IncTick();
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

  Delay(600); // Wait a short time (allows messages to get out before shutting down.
  cli(); // Stop interrupts

  Delay(250); //Wait to ensure all interrupts routines stopped
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

uint32_t millis() {
  return GetTick();
}

/**
 * Send an "ok" message to the host, indicating
 * that a command was successfully processed.
 *
 * If ADVANCED_OK is enabled also include:
 *   N<int>  Line number of the command, if any
 *   P<int>  Planner space remaining
 *   B<int>  Block queue space remaining
 */
void ok_to_send() {
  refresh_cmd_timeout();
  if (!send_ok[cmd_queue_index_r]) return;
  SERIAL_PROTOCOLPGM(MSG_OK);
  #if ENABLED(ADVANCED_OK)
    char* p = command_queue[cmd_queue_index_r];
    if (*p == 'N') {
      SERIAL_PROTOCOL(' ');
      SERIAL_ECHO(*p++);
      while (NUMERIC_SIGNED(*p))
        SERIAL_ECHO(*p++);
    }
    SERIAL_PROTOCOLPGM(" P"); SERIAL_PROTOCOL(int(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
    SERIAL_PROTOCOLPGM(" B"); SERIAL_PROTOCOL(BUFSIZE - commands_in_queue);
  #endif
  SERIAL_EOL();
}

/**
  * @brief This function provides accurate delay (in ms) based on a variable incremented.
  * @note In the default implementation , SysTick timer is the source of time base.
  *       It is used to generate interrupts at regular time intervals where uwTick
  *       is incremented.
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0U;
  tickstart = GetTick();
  while((GetTick() - tickstart) < Delay)
  {
  }
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
