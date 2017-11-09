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

/* USER CODE BEGIN Includes */
#include "CommandParser.h"
#include "Planner.h"
#include "serial.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t rx_buf[RXBUF_LEN], tx_buf[TXBUF_LEN];
/* xx_i - counter of input bytes (tx - pushed for transmit, rx - received)
   xx_o - counter of output bytes (tx - transmitted, rx - parsed)
   xx_e - counter of echoed bytes */
volatile uint16_t rx_i = 0, tx_o = 0;
uint16_t rx_o = 0, rx_e = 0, tx_i = 0;
volatile uint8_t tx_busy = 0;

uint8_t commands_in_queue = 0; // Count of commands in the queue
static uint8_t cmd_queue_index_r = 0, // Ring buffer read position
               cmd_queue_index_w = 0; // Ring buffer write position
static char command_queue[BUFSIZE][MAX_CMD_SIZE];

bool Running = true;

const char axis_codes[XYZE] = { 'X', 'Y', 'Z', 'E' };

/**
 * Cartesian Current Position
 *   Used to track the logical position as moves are queued.
 *   Used by 'line_to_current_position' to do a move after changing it.
 *   Used by 'SYNC_PLAN_POSITION_KINEMATIC' to update 'planner.position'.
 */
float current_position[XYZE] = { 0.0 };

/**
 * Cartesian Destination
 *   A temporary position, usually applied to 'current_position'.
 *   Set with 'gcode_get_destination' or 'set_destination_to_current'.
 *   'line_to_destination' sets 'current_position' to 'destination'.
 */
float destination[XYZE] = { 0.0 };

// Initialized by settings.load()
#define AXIS_RELATIVE_MODES {false, false, false, false}
bool axis_relative_modes[] = AXIS_RELATIVE_MODES;

// Software Endstops are based on the configured limits.
#if HAS_SOFTWARE_ENDSTOPS
  bool soft_endstops_enabled = true;
#endif
float soft_endstop_min[XYZ] = { X_MIN_BED, Y_MIN_BED, Z_MIN_POS },
      soft_endstop_max[XYZ] = { X_MAX_BED, Y_MAX_BED, Z_MAX_POS };

// Relative Mode. Enable with G91, disable with G90.
static bool relative_mode = false;

float feedrate_mm_s = MMM_TO_MMS(1500.0);
static float saved_feedrate_mm_s;

volatile uint32_t counter = 0;

int16_t feedrate_percentage = 100, saved_feedrate_percentage,
    flow_percentage[EXTRUDERS] = ARRAY_BY_EXTRUDERS1(100);

// Initialized by settings.load()
bool axis_relative_modes[] = AXIS_RELATIVE_MODES,
     volumetric_enabled;
float filament_size[EXTRUDERS], volumetric_multiplier[EXTRUDERS];

// The active extruder (tool). Set with T<extruder> command.
uint8_t active_extruder = 0;

#define XY_PROBE_FEEDRATE_MM_S PLANNER_XY_FEEDRATE()

/**
 * axis_homed
 *   Flags that each linear axis was homed.
 *   XYZ on cartesian, ABC on delta, ABZ on SCARA.
 *
 * axis_known_position
 *   Flags that the position is known in each linear axis. Set when homed.
 *   Cleared whenever a stepper powers off, potentially losing its position.
 */
bool axis_homed[XYZ] = { false }, axis_known_position[XYZ] = { false };

// Number of characters read in the current line of serial input
static int serial_count = 0;

/**
 * GCode line number handling. Hosts may opt to include line numbers when
 * sending commands to Marlin, and lines will be checked for sequentiality.
 * M110 N<int> sets the current line number.
 */
static long gcode_N, gcode_LastN;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_LPUART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define XYZ_CONSTS_FROM_CONFIG(type, array, CONFIG) \
  static const type array##_P[XYZ] = { X_##CONFIG, Y_##CONFIG, Z_##CONFIG }; \
  static inline type array(AxisEnum axis) { return array##_P[axis]; } \
  typedef void __void_##CONFIG##__

XYZ_CONSTS_FROM_CONFIG(float, base_home_pos,  HOME_POS);
XYZ_CONSTS_FROM_CONFIG(float, max_length,     MAX_LENGTH);
XYZ_CONSTS_FROM_CONFIG(float, home_bump_mm,   HOME_BUMP_MM);
XYZ_CONSTS_FROM_CONFIG(signed char, home_dir, HOME_DIR);

void transmit(UART_HandleTypeDef *huart, uint8_t byte)
{
    tx_buf[TXBUF_MSK & tx_i] = byte;
    tx_i++;
    tx_busy = 1;
    __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);
}

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void sync_plan_position() {
  planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);
}
#define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()

/**
 * Set XYZE destination and feedrate from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void gcode_get_destination() {
  LOOP_XYZE(i) {
    if (parser.seen(axis_codes[i]))
      destination[i] = parser.value_axis_units((AxisEnum)i) + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
    else
      destination[i] = current_position[i];
  }

  if (parser.linearval('F') > 0.0)
    feedrate_mm_s = MMM_TO_MMS(parser.value_feedrate());
}

#if HAS_SOFTWARE_ENDSTOPS

  /**
   * Constrain the given coordinates to the software endstops.
   */

  // NOTE: This makes no sense for delta beds other than Z-axis.
  //       For delta the X/Y would need to be clamped at
  //       DELTA_PRINTABLE_RADIUS from center of bed, but delta
  //       now enforces is_position_reachable for X/Y regardless
  //       of HAS_SOFTWARE_ENDSTOPS, so that enforcement would be
  //       redundant here.

  void clamp_to_software_endstops(float target[XYZ]) {
    if (!soft_endstops_enabled) return;
    #if ENABLED(MIN_SOFTWARE_ENDSTOPS)
      #if DISABLED(DELTA)
        NOLESS(target[X_AXIS], soft_endstop_min[X_AXIS]);
        NOLESS(target[Y_AXIS], soft_endstop_min[Y_AXIS]);
      #endif
      NOLESS(target[Z_AXIS], soft_endstop_min[Z_AXIS]);
    #endif
    #if ENABLED(MAX_SOFTWARE_ENDSTOPS)
      #if DISABLED(DELTA)
        NOMORE(target[X_AXIS], soft_endstop_max[X_AXIS]);
        NOMORE(target[Y_AXIS], soft_endstop_max[Y_AXIS]);
      #endif
      NOMORE(target[Z_AXIS], soft_endstop_max[Z_AXIS]);
    #endif
  }

#endif

  /**
   * Move the planner to the position stored in the destination array, which is
   * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
   */
  inline void line_to_destination(const float fr_mm_s) {
    planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], destination[Z_AXIS], destination[E_AXIS], fr_mm_s, active_extruder);
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
      if (current_position[X_AXIS] == destination[X_AXIS] && current_position[Y_AXIS] == destination[Y_AXIS])
        line_to_destination();
      else {
        const float fr_scaled = MMS_SCALED(feedrate_mm_s);
            line_to_destination(fr_scaled);
      }
    return false;
  }

  inline void set_current_to_destination() { COPY(current_position, destination); }
  inline void set_destination_to_current() { COPY(destination, current_position); }

/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 */
void prepare_move_to_destination() {
  clamp_to_software_endstops(destination);
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

//
// Prepare to do endstop or probe moves
// with custom feedrates.
//
//  - Save current feedrates
//  - Reset the rate multiplier
//  - Reset the command timeout
//  - Enable the endstops (for endstop moves)
//
static void setup_for_endstop_or_probe_move() {
  saved_feedrate_mm_s = feedrate_mm_s;
  saved_feedrate_percentage = feedrate_percentage;
  feedrate_percentage = 100;
  refresh_cmd_timeout();
}

/**
 * Feed rates are often configured with mm/m
 * but the planner and stepper like mm/s units.
 */
static const float homing_feedrate_mm_s[] = {
  MMM_TO_MMS(HOMING_FEEDRATE_XY), MMM_TO_MMS(HOMING_FEEDRATE_XY),
  MMM_TO_MMS(HOMING_FEEDRATE_Z), 0
};
FORCE_INLINE float homing_feedrate(const AxisEnum a) { return homing_feedrate_mm_s[a]; }

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
inline void line_to_current_position() {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], feedrate_mm_s, active_extruder);
}

/**
 *  Plan a move to (X, Y, Z) and set the current_position
 *  The final current_position may not be the one that was requested
 */
void do_blocking_move_to(const float &lx, const float &ly, const float &lz, const float &fr_mm_s/*=0.0*/) {
  const float old_feedrate_mm_s = feedrate_mm_s;

  // If Z needs to raise, do it before moving XY
  if (current_position[Z_AXIS] < lz) {
    feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS);
    current_position[Z_AXIS] = lz;
    line_to_current_position();
  }

  feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
  current_position[X_AXIS] = lx;
  current_position[Y_AXIS] = ly;
  line_to_current_position();

  // If Z needs to lower, do it after moving XY
  if (current_position[Z_AXIS] > lz) {
    feedrate_mm_s = fr_mm_s ? fr_mm_s : homing_feedrate(Z_AXIS);
    current_position[Z_AXIS] = lz;
    line_to_current_position();
  }

  stepper.synchronize();

  feedrate_mm_s = old_feedrate_mm_s;
}

void do_blocking_move_to_z(const float &lz, const float &fr_mm_s = 0.0f) {
  do_blocking_move_to(current_position[X_AXIS], current_position[Y_AXIS], lz, fr_mm_s);
}

#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

/**
 * Home an individual linear axis
 */
static void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0.0) {

  // Tell the planner we're at Z=0
  current_position[axis] = 0;

  sync_plan_position();
  current_position[axis] = distance;
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS], fr_mm_s ? fr_mm_s : homing_feedrate(axis), active_extruder);

  stepper.synchronize();

  endstops.hit_on_purpose();
}

/**
 * Some planner shorthand inline functions
 */
inline float get_homing_bump_feedrate(const AxisEnum axis) {
  static const uint8_t homing_bump_divisor[] = HOMING_BUMP_DIVISOR;
  uint8_t hbd = homing_bump_divisor[axis];
  if (hbd < 1) {
    hbd = 10;
  }
  return homing_feedrate(axis) / hbd;
}

/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * DELTA should wait until all homing is done before setting the XYZ
 * current_position to home, because homing is a single operation.
 * In the case where the axis positions are already known and previously
 * homed, DELTA could home to X or Y individually by moving either one
 * to the center. However, homing Z always homes XY and Z.
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * current_position to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 *
 * Callers must sync the planner position after calling this!
 */
static void set_axis_is_at_home(const AxisEnum axis) {
  axis_known_position[axis] = axis_homed[axis] = true;

  current_position[axis] = LOGICAL_POSITION(base_home_pos(axis), axis);
}

static void homeaxis(const AxisEnum axis) {

  #define CAN_HOME(A) \
    (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
  if (!CAN_HOME(X) && !CAN_HOME(Y) && !CAN_HOME(Z)) return;

  const int axis_home_dir =
    home_dir(axis);

  // Fast move towards endstop until triggered
  do_homing_move(axis, 1.5 * max_length(axis) * axis_home_dir);

  // When homing Z with probe respect probe clearance
  const float bump = axis_home_dir * (
    home_bump_mm(axis)
  );

  // If a second homing move is configured...
  if (bump) {
    // Move away from the endstop by the axis HOME_BUMP_MM
    do_homing_move(axis, -bump);

    // Slow move towards endstop until triggered
    do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
  }

  // For cartesian/core machines,
  // set the axis to its home position
  set_axis_is_at_home(axis);
  sync_plan_position();

  destination[axis] = current_position[axis];

} // homeaxis()

static void clean_up_after_endstop_or_probe_move() {
  feedrate_mm_s = saved_feedrate_mm_s;
  feedrate_percentage = saved_feedrate_percentage;
  refresh_cmd_timeout();
}

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */
inline void gcode_G28(const bool always_home_all) {

  // Wait for planner moves to finish!
  stepper.synchronize();

  setup_for_endstop_or_probe_move();
  endstops.enable(true); // Enable endstops for next homing move

    const bool homeX = always_home_all || parser.seen('X'),
               homeY = always_home_all || parser.seen('Y'),
               homeZ = always_home_all || parser.seen('Z'),
               home_all = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    set_destination_to_current();

    #if Z_HOME_DIR > 0  // If homing away from BED do Z first

      if (home_all || homeZ) {
        HOMEAXIS(Z);
      }

    #else

      if (home_all || homeX || homeY) {
        // Raise Z before homing any other axes and z is not already high enough (never lower z)
        destination[Z_AXIS] = LOGICAL_Z_POSITION(Z_HOMING_HEIGHT);
        if (destination[Z_AXIS] > current_position[Z_AXIS]) {

          do_blocking_move_to_z(destination[Z_AXIS]);
        }
      }

    #endif

    // Home X
    if (home_all || homeX) {

        HOMEAXIS(X);

    }

    // Home Y
    if (home_all || homeY) {
      HOMEAXIS(Y);
    }

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0
      if (home_all || homeZ) {
        HOMEAXIS(Z);
      } // home_all || homeZ
    #endif // Z_HOME_DIR < 0

    SYNC_PLAN_POSITION_KINEMATIC();

  endstops.not_homing();

  clean_up_after_endstop_or_probe_move();
} // G28

inline void sync_plan_position_e() { planner.set_e_position_mm(current_position[E_AXIS]); }

/**
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {
  bool didXYZ = false,
       didE = parser.seenval('E');

  if (!didE) stepper.synchronize();

  LOOP_XYZE(i) {
    if (parser.seenval(axis_codes[i])) {
        const float v = parser.value_axis_units((AxisEnum)i);

        current_position[i] = v;

        if (i != E_AXIS) {
          didXYZ = true;
        }
    }
  }
  if (didXYZ)
    SYNC_PLAN_POSITION_KINEMATIC();
  else if (didE)
    sync_plan_position_e();
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

      case 28: // G28: Home all axes, one at a time
        gcode_G28(false);
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
  #if HAS_POWER_SWITCH
    OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE);
  #endif
}

/**
 * Stepper Reset (RigidBoard, et.al.)
 */
#if HAS_STEPPER_RESET
  void disableStepperDrivers() {
    HAL_GPIO_WritePin(GPIO?, GPIO_PIN_?, GPIO_PIN_?SET);
    OUT_WRITE(STEPPER_RESET_PIN, LOW);  // drive it down to hold in reset motor driver chips
  }
  void enableStepperDrivers() {
	  GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = GPIO_PIN_?;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIO?, &GPIO_InitStruct);

	  SET_INPUT(STEPPER_RESET_PIN);
  }  // set to input, which allows it to be pulled high by pullups
#endif

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

  #if HAS_STEPPER_RESET
    disableStepperDrivers();
  #endif

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

  ZERO(current_position);

  // Vital to init stepper/planner equivalent for current_position
  SYNC_PLAN_POSITION_KINEMATIC();

  thermalManager.init();    // Initialize temperature loop

  #if ENABLED(USE_WATCHDOG)
    watchdog_init();
  #endif

  stepper.init();    // Initialize stepper, this enables interrupts!

  #if HAS_STEPPER_RESET
    enableStepperDrivers();
  #endif

  #if ENABLED(RGB_LED) || ENABLED(RGBW_LED)
    SET_OUTPUT(RGB_LED_R_PIN);
    SET_OUTPUT(RGB_LED_G_PIN);
    SET_OUTPUT(RGB_LED_B_PIN);
    #if ENABLED(RGBW_LED)
      SET_OUTPUT(RGB_LED_W_PIN);
    #endif
  #endif

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    setup_endstop_interrupts();
  #endif
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
  MX_USART2_UART_Init();
  MX_LPUART1_UART_Init();

  /* USER CODE BEGIN 2 */
  setup();
  /* Enable lpusart 1 receive IRQ */
  __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);
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
      while (rx_i != rx_e) {
          /* echo here */
          transmit(&huart2, rx_buf[RXBUF_MSK & rx_e]);
          rx_e++;
      }
      while (rx_i != rx_o) {
          /* parse here */
          /* ... */
          rx_o++;
      }
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

/* LPUART1 init function */
static void MX_LPUART1_UART_Init(void)
{

  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
//  char *msg = "CONNECTING...\n\r";
//  HAL_UART_Transmit_IT(&huart2, (uint8_t*)msg, strlen(msg));

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
//  BSP_LED_Toggle(LED2);
//  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
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

void SysTick_Handler(void) {
  counter++;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
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
