/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include <stm32l053xx.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
// For Inverting Stepper Enable Pins (Active Low) use 0, Non Inverting (Active High) use 1
// :{ 0:'Low', 1:'High' }
#define MOTOR_ENABLE_ON GPIO::GPIO_PIN_RESET

// Transfer Buffer Size
// To save 386 bytes of PROGMEM (and TX_BUFFER_SIZE+3 bytes of RAM) set to 0.
// To buffer a simple "ok" you need 4 bytes.
// For ADVANCED_OK (M105) you need 32 bytes.
// For debug-echo: 128 bytes for the optimal speed.
// Other output doesn't need to be that speedy.
// :[0, 2, 4, 8, 16, 32, 64, 128, 256]
constexpr uint8_t TX_BUFFER_SIZE = 128U;

namespace Thermoprinter {
	extern bool Running;
	inline bool IsRunning() { return  Running; }
	inline bool IsStopped() { return !Running; }

	void idle();

	namespace BSP {
		void MX_GPIO_Init(void);
		void setup_powerhold();
		void setup();
		void MspInit(void);
		void Init(void);
	}

	void kill();

	void Error_Handler(const char * const, const int32_t);

	#if defined(HOST_KEEPALIVE_FEATURE) && (HOST_KEEPALIVE_FEATURE > 0)
		MarlinBusyState busy_state = NOT_BUSY;
		static millis_t next_busy_signal_ms = 0;
		uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
	#else
	  	#define host_keepalive() NOOP
	#endif

}

namespace RuntimeSettings {
	extern bool axis_relative_modes;
	extern bool relative_mode;

	extern float32_t feedrate_mm_s;

	extern int16_t feedrate_percentage;
	extern int16_t saved_feedrate_percentage;
}

enum EndstopEnum {
    MOTOR_FAULT,
	LO_BAT,
	VH_ON_CTRL,
	HEAD_UP,
	PAPER_END,
	OVER_HEAT
};

constexpr float MMM_TO_MMS(float32_t MM_M) {
	return ((MM_M)/60.0);
}
#define MMS_SCALED(MM_S) ((MM_S)*RuntimeSettings::feedrate_percentage*0.01)

#define IS_CARTESIAN 1

// Minimum planner junction speed. Sets the default minimum speed the planner plans for at the end
// of the buffer and all stops. This should not be much greater than zero and should only be changed
// if unwanted behavior is observed on a user's machine when running at very slow speeds.
#define MINIMUM_PLANNER_SPEED 0.05F // (mm/sec)

/**
 * Select which serial port on the board will be used for communication with the host.
 * This allows the connection of wireless adapters (for instance) to non-default port pins.
 *
 * :[0, 1, 2]
 */
#define SERIAL_PORT 0

/**
 * This setting determines the communication speed of the printer.
 *
 * 115200 works in most cases, but you might try a lower speed if
 * you commonly experience drop-outs during host printing.
 *
 * :[2400, 9600, 19200, 38400, 57600, 115200]
 */
#define BAUDRATE 9600

// Enable the Bluetooth serial interface
//#define BLUETOOTH

//#define EEPROM_SETTINGS   // Enable for M500 and M501 commands

/**
 * Default Steps Per Unit (steps/mm)
 */
 // default steps per unit for RigidBot with standard hardware
#define DEFAULT_STEPS_PER_UNIT   16

/**
 * Default Max Feed Rate (mm/s)
 */
#define DEFAULT_MAX_FEEDRATE          62.5

/**
 * Default Max Acceleration (change/s) change = mm/s
 * (Maximum start speed for accelerated moves)
 */
#define DEFAULT_MAX_ACCELERATION      11.75

/**
 * Default Acceleration (change/s) change = mm/s
 */
#define DEFAULT_ACCELERATION          10     // acceleration for printing moves
#define DEFAULT_TRAVEL_ACCELERATION   10    // travel (non printing) moves

#define DEFAULT_MINIMUMFEEDRATE       0.0     // minimum feedrate
#define DEFAULT_MINTRAVELFEEDRATE     0.0

// minimum time in microseconds that a movement needs to take if the buffer is emptied.
#define DEFAULT_MINSEGMENTTIME        20000

/**
 * Default Jerk (mm/s)
 *
 * "Jerk" specifies the minimum speed change that requires acceleration.
 * When changing speed and direction, if the difference is less than the
 * value set here, it may happen instantaneously.
 */
#define DEFAULT_JERK                 2.0

// If you want endstops to stay on (by default) even when not homing
// enable this option. Override at any time with M120, M121.
#define ENDSTOPS_ALWAYS_ON_DEFAULT

// Mechanical endstop with COM to ground and NC to Signal uses "false" here (most common setup).
#define MOTOR_FAULT_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define LO_BAT_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define VH_ON_CTRL_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define HEAD_UP_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define PAPER_END_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.
#define OVER_HEAT_ENDSTOP_INVERTING false // set to true to invert the logic of the endstop.

// Enable this feature if all enabled endstop pins are interrupt-capable.
// This will remove the need to poll the interrupt pins, saving many CPU cycles.
#define ENDSTOP_INTERRUPTS_FEATURE

// By default DRV step driver require an active high signal. However, some high power drivers require an active low signal as step.
#define INVERT_MOTOR_STEP_PIN GPIO::GPIO_PIN_RESET

// The minimum pulse width (in µs) for stepping a stepper.
// Set this if you find stepping unreliable, or if using a very fast CPU.
#define MINIMUM_STEPPER_PULSE 4 // (µs) The smallest stepper pulse allowed

// The minimal temperature defines the temperature below which the printhead will not be enabled It is used
// to check that the wiring to the thermistor is not broken.
// Otherwise this would lead to the heater being powered on all the time.
#define PRINTHEAD_MINTEMP -10

// When temperature exceeds max temp, your printhead will be switched off.
// This feature exists to protect your printhead from overheating accidentally, but *NOT* from thermistor short/failure!
// You should use MINTEMP for thermistor short/failure protection.
#define PRINTHEAD_MAXTEMP 65

// The minimal voltage defines the voltage below which the printing will not be enabled It is used
// to check that the wiring to the battery is not broken.
// Otherwise this would lead to the printer being powered on all the time.
#define BATTERY_MINVOLT 6

// When battery exceeds max volt, your charger will be switched off.
// TODO: fix the following text. This feature exists to protect your battery from overcharging accidentally, but *NOT* from charger short/failure!
// You should use MINVOLT for charger short/failure protection.
#define BATTERY_MAXVOLT 8.5

#define MSG_T_MAXTEMP                       "MAXTEMP triggered"
#define MSG_T_MINTEMP                       "MINTEMP triggered"
#ifndef MSG_ERR_MAXTEMP
  #define MSG_ERR_MAXTEMP                     "Err: MAXTEMP"
#endif
#ifndef MSG_ERR_MINTEMP
  #define MSG_ERR_MINTEMP                     "Err: MINTEMP"
#endif

#define MSG_THERMOPRINTER "Thermoprinter"

#define SHORT_BUILD_VERSION "0.1"

#define MSG_OK                              "ok"

#define MSG_ERR_KILLED                      "Printer halted. kill() called!"

#define MSG_RESEND                          "Resend: "

#define MSG_ERR_LINE_NO                     "Line Number is not Last Line Number+1, Last Line: "

#define MSG_STOPPED_HEATER                  ", system stopped! Heater_ID: "

uint32_t GetTick(void);

namespace Timers {
	void Delay(__IO uint32_t delay_ms);
}

extern "C" {
	void SysTick_Handler(void);
}

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
