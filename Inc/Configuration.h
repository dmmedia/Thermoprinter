/*
 * Configuration.h
 *
 *  Created on: 27. nov 2017
 *      Author: Den
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#define MOTOR_STEP_PORT		GPIOA
#define MOTOR_STEP_PIN		GPIO_PIN_2
#define MOTOR_DIR_PORT		GPIOA
#define MOTOR_DIR_PIN		GPIO_PIN_3
#define MOTOR_ENABLE_PORT	GPIOA
#define MOTOR_ENABLE_PIN	GPIO_PIN_1

//
// Limit Switches
//
#define MOTOR_FAULT_PIN		GPIO_PIN_6
#define MOTOR_FAULT_PORT	GPIOA

#define OVER_HEAT_PIN		GPIO_PIN_4
#define OVER_HEAT_PORT		GPIOC

#define LO_BAT_PIN			GPIO_PIN_5
#define LO_BAT_PORT		GPIOC

#define VH_ON_CTRL_PIN		GPIO_PIN_13
#define VH_ON_CTRL_PORT		GPIOC

#define PAPER_END_PIN		GPIO_PIN_1
#define PAPER_END_PORT		GPIOB

#define HEAD_UP_PIN			GPIO_PIN_2
#define HEAD_UP_PORT		GPIOB

//
// Misc. Functions
//
#undef PS_ON_PIN
#define PS_ON_PIN          -1
#define PS_ON_PORT		   GPIOA
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
#define ADC1_CHANNEL_VOLTAGE_PIN               GPIO_PIN_0

#define ADC1_CHANNEL_TEMPERATURE_GPIO_PORT         GPIOA
#define ADC1_CHANNEL_TEMPERATURE_PIN               GPIO_PIN_5

/* Definition of ADCx NVIC resources */
#define ADC1_IRQn                       ADC1_COMP_IRQn
#define ADC1_IRQHandler                 ADC1_COMP_IRQHandler


#define FASTER_COMMAND_PARSER

#define RXBUF_LEN            128 // must be power of 2
#define TXBUF_LEN            128 // must be power of 2
#define RXBUF_MSK            (RXBUF_LEN-1)
#define TXBUF_MSK            (TXBUF_LEN-1)

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
 * :{ '0': "Not used", '1':"100k / 4.7k - EPCOS", '2':"200k / 4.7k - ATC Semitec 204GT-2", '3':"Mendel-parts / 4.7k", '4':"10k !! do not use for a hotend. Bad resolution at high temp. !!", '5':"100K / 4.7k - ATC Semitec 104GT-2 (Used in ParCan & J-Head)", '6':"100k / 4.7k EPCOS - Not as accurate as Table 1", '7':"100k / 4.7k Honeywell 135-104LAG-J01", '8':"100k / 4.7k 0603 SMD Vishay NTCS0603E3104FXT", '9':"100k / 4.7k GE Sensing AL03006-58.2K-97-G1", '10':"100k / 4.7k RS 198-961", '11':"100k / 4.7k beta 3950 1%", '12':"100k / 4.7k 0603 SMD Vishay NTCS0603E3104FXT (calibrated for Makibox hot bed)", '13':"100k Hisens 3950  1% up to 300°C for hotend 'Simple ONE ' & hotend 'All In ONE'", '20':"PT100 (Ultimainboard V2.x)", '51':"100k / 1k - EPCOS", '52':"200k / 1k - ATC Semitec 204GT-2", '55':"100k / 1k - ATC Semitec 104GT-2 (Used in ParCan & J-Head)", '60':"100k Maker's Tool Works Kapton Bed Thermistor beta=3950", '66':"Dyze Design 4.7M High Temperature thermistor", '70':"the 100K thermistor found in the bq Hephestos 2", '71':"100k / 4.7k Honeywell 135-104LAF-J01", '147':"Pt100 / 4.7k", '1047':"Pt1000 / 4.7k", '110':"Pt100 / 1k (non-standard)", '1010':"Pt1000 / 1k (non standard)", '-3':"Thermocouple + MAX31855 (only for sensor 0)", '-2':"Thermocouple + MAX6675 (only for sensor 0)", '-1':"Thermocouple + AD595",'998':"Dummy 1", '999':"Dummy 2" }
 */
#define TEMP_SENSOR_0 998 // DGlass3D = 5; RigidBot = 1; 3DSv6 = 5

// Invert the stepper direction. Change (or reverse the motor connector) if an paper goes the wrong way.
#define INVERT_MOTOR_DIR GPIO_PIN_RESET



#endif /* CONFIGURATION_H_ */
