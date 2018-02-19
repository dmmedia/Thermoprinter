#pragma once

/*
 * Configuration.h
 *
 *  Created on: 27. nov 2017
 *      Author: Den
 */

#define MOTOR_STEP_PORT		GPIOA
#define MOTOR_STEP_PIN		GPIO::GPIO_PIN_2
#define MOTOR_DIR_PORT		GPIOA
#define MOTOR_DIR_PIN		GPIO::GPIO_PIN_3
#define MOTOR_ENABLE_PORT	GPIOA
#define MOTOR_ENABLE_PIN	GPIO::GPIO_PIN_1

//
// Limit Switches
//
#define MOTOR_FAULT_PIN		GPIO::GPIO_PIN_6
#define MOTOR_FAULT_PORT	GPIOA

#define OVER_HEAT_PIN		GPIO::GPIO_PIN_4
#define OVER_HEAT_PORT		GPIOC

#define LO_BAT_PIN			GPIO::GPIO_PIN_5
#define LO_BAT_PORT		GPIOC

#define VH_ON_CTRL_PIN		GPIO::GPIO_PIN_13
#define VH_ON_CTRL_PORT		GPIOC

#define PAPER_END_PIN		GPIO::GPIO_PIN_1
#define PAPER_END_PORT		GPIOB

#define HEAD_UP_PIN			GPIO::GPIO_PIN_2
#define HEAD_UP_PORT		GPIOB

//
// Misc. Functions
//
#undef PS_ON_PIN
#define PS_ON_PIN          GPIO::NO_PIN
#define PS_ON_PORT		   GPIO::NO_PORT
//#define PS_ON_PIN   GPIO_PIN_1
//#define PS_ON_PORT	GPIOA
//#define POWER_SUPPLY 0

//
// Temperature Sensors
//
#define TEMP_0_PIN          GPIO_PORT_5  // Analog Input
#define TEMP_0_PORT			GPIOA

#define FILWIDTH_PIN     GPIO_PIN_0   // should be Analog Input
#define FILWIDTH_PORT    GPIOB

#define MAX_STEP_FREQUENCY 1000

/* ## Definition of ADC related resources ################################### */
/* Definition of ADCx channels */
#define ADC1_CHANNEL_VOLTAGE                   ADC_CHANNEL_8
#define ADC1_CHANNEL_TEMPERATURE               ADC_CHANNEL_5

/* Definition of ADCx channels pins */
#define ADC1_CHANNEL_VOLTAGE_GPIO_PORT         GPIOB
#define ADC1_CHANNEL_VOLTAGE_PIN               GPIO::GPIO_PIN_0

#define ADC1_CHANNEL_TEMPERATURE_GPIO_PORT         GPIOA
#define ADC1_CHANNEL_TEMPERATURE_PIN               GPIO::GPIO_PIN_5

/* Definition of ADCx NVIC resources */
#define ADC1_IRQn                       ADC1_COMP_IRQn
#define ADC1_IRQHandler                 ADC1_COMP_IRQHandler


#define RXBUF_LEN            128 // must be power of 2
#define TXBUF_LEN            128U // must be power of 2
#define RXBUF_MSK            (RXBUF_LEN-1)
#define TXBUF_MSK            (TXBUF_LEN - 1U)

/**
 * Temperature sensors available:
 *
 *    -3 : thermocouple with MAX31855 (only for sensor 0)
 *    -2 : thermocouple with MAX6675 (only for sensor 0)
 *    -1 : thermocouple with AD595
 *     0 : not used
 *     1 : 100k thermistor - best choice for EPCOS 100k (4.7k pullup)
 *     2 : 200k thermistor - ATC Semitec 204GT-2 (4.7k pullup)
 *     3 : Mendel-parts thermistor (4.7k pullup)
 *     4 : 10k thermistor !! do not use it for a hotend. It gives bad resolution at high temp. !!
 *     5 : 100K thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (4.7k pullup)
 *     6 : 100k EPCOS - Not as accurate as table 1 (created using a fluke thermocouple) (4.7k pullup)
 *     7 : 100k Honeywell thermistor 135-104LAG-J01 (4.7k pullup)
 *    71 : 100k Honeywell thermistor 135-104LAF-J01 (4.7k pullup)
 *     8 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup)
 *     9 : 100k GE Sensing AL03006-58.2K-97-G1 (4.7k pullup)
 *    10 : 100k RS thermistor 198-961 (4.7k pullup)
 *    11 : 100k beta 3950 1% thermistor (4.7k pullup)
 *    12 : 100k 0603 SMD Vishay NTCS0603E3104FXT (4.7k pullup) (calibrated for Makibox hot bed)
 *    13 : 100k Hisens 3950  1% up to 300°C for hotend "Simple ONE " & "Hotend "All In ONE"
 *    20 : the PT100 circuit found in the Ultimainboard V2.x
 *    60 : 100k Maker's Tool Works Kapton Bed Thermistor beta=3950
 *    66 : 4.7M High Temperature thermistor from Dyze Design
 *    70 : the 100K thermistor found in the bq Hephestos 2
 *    75 : 100k Generic Silicon Heat Pad with NTC 100K MGB18-104F39050L32 thermistor
 *
 *       1k ohm pullup tables - This is atypical, and requires changing out the 4.7k pullup for 1k.
 *                              (but gives greater accuracy and more stable PID)
 *    51 : 100k thermistor - EPCOS (1k pullup)
 *    52 : 200k thermistor - ATC Semitec 204GT-2 (1k pullup)
 *    55 : 100k thermistor - ATC Semitec 104GT-2 (Used in ParCan & J-Head) (1k pullup)
 *
 *  1047 : Pt1000 with 4k7 pullup
 *  1010 : Pt1000 with 1k pullup (non standard)
 *   147 : Pt100 with 4k7 pullup
 *   110 : Pt100 with 1k pullup (non standard)
 *
 *         Use these for Testing or Development purposes. NEVER for production machine.
 *   998 : Dummy Table that ALWAYS reads 25°C or the temperature defined below.
 *   999 : Dummy Table that ALWAYS reads 100°C or the temperature defined below.
 *
 * :{
 *    '0': "Not used",
 *    '1':"100k / 4.7k - EPCOS",
 *    '2':"200k / 4.7k - ATC Semitec 204GT-2",
 *    '3':"Mendel-parts / 4.7k",
 *    '4':"10k !! do not use for a hotend. Bad resolution at high temp. !!",
 *    '5':"100K / 4.7k - ATC Semitec 104GT-2 (Used in ParCan & J-Head)",
 *    '6':"100k / 4.7k EPCOS - Not as accurate as Table 1",
 *    '7':"100k / 4.7k Honeywell 135-104LAG-J01",
 *    '8':"100k / 4.7k 0603 SMD Vishay NTCS0603E3104FXT",
 *    '9':"100k / 4.7k GE Sensing AL03006-58.2K-97-G1",
 *   '10':"100k / 4.7k RS 198-961",
 *   '11':"100k / 4.7k beta 3950 1%",
 *   '12':"100k / 4.7k 0603 SMD Vishay NTCS0603E3104FXT (calibrated for Makibox hot bed)",
 *   '13':"100k Hisens 3950  1% up to 300°C for hotend 'Simple ONE ' & hotend 'All In ONE'",
 *   '20':"PT100 (Ultimainboard V2.x)",
 *   '51':"100k / 1k - EPCOS",
 *   '52':"200k / 1k - ATC Semitec 204GT-2",
 *   '55':"100k / 1k - ATC Semitec 104GT-2 (Used in ParCan & J-Head)",
 *   '60':"100k Maker's Tool Works Kapton Bed Thermistor beta=3950",
 *   '66':"Dyze Design 4.7M High Temperature thermistor",
 *   '70':"the 100K thermistor found in the bq Hephestos 2",
 *   '71':"100k / 4.7k Honeywell 135-104LAF-J01",
 *  '147':"Pt100 / 4.7k",
 * '1047':"Pt1000 / 4.7k",
 *  '110':"Pt100 / 1k (non-standard)",
 * '1010':"Pt1000 / 1k (non standard)",
 *   '-3':"Thermocouple + MAX31855 (only for sensor 0)",
 *   '-2':"Thermocouple + MAX6675 (only for sensor 0)",
 *   '-1':"Thermocouple + AD595",
 *  '998':"Dummy 1",
 *  '999':"Dummy 2"
 * }
 */
#define TEMP_SENSOR_0 998 // DGlass3D = 5; RigidBot = 1; 3DSv6 = 5

// Invert the stepper direction. Change (or reverse the motor connector) if an paper goes the wrong way.
#define INVERT_MOTOR_DIR GPIO::GPIO_PIN_RESET

// The ASCII buffer for serial input
#define MAX_CMD_SIZE 128 // to fit 96 hex symbols + P0 command
#define BUFSIZE 4U

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
constexpr int8_t MINIMUM_STEPPER_PULSE = 4; // (µs) The smallest stepper pulse allowed

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

