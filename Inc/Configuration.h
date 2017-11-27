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
//#define PS_ON_PIN   GPIO_PIN_1
//#define PS_ON_PORT	GPIOA

//
// Temperature Sensors
//
#define TEMP_0_PIN          GPIO_PORT_5  // Analog Input
#define TEMP_0_PORT			GPIOA

#define FILWIDTH_PIN     GPIO_PIN_0   // should be Analog Input
#define FILWIDTH_PORT    GPIOB

#define MAX_STEP_FREQUENCY 1000

/* ## Definition of ADC related resources ################################### */
/* Definition of ADCx clock resources */
#define ADC1_CLK_ENABLE()               __ADC1_CLK_ENABLE()

#define ADC1_FORCE_RESET()              __ADC1_FORCE_RESET()
#define ADC1_RELEASE_RESET()            __ADC1_RELEASE_RESET()

/* Definition of ADCx channels */
#define ADC1_CHANNEL_VOLTAGE                   ADC_CHANNEL_8
#define ADC1_CHANNEL_TEMPERATURE               ADC_CHANNEL_5

/* Definition of ADCx channels pins */
#define ADC1_CHANNEL_VOLTAGE_GPIO_CLK_ENABLE() __GPIOB_CLK_ENABLE()
#define ADC1_CHANNEL_VOLTAGE_GPIO_PORT         GPIOB
#define ADC1_CHANNEL_VOLTAGE_PIN               GPIO_PIN_0

#define ADC1_CHANNEL_TEMPERATURE_GPIO_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define ADC1_CHANNEL_TEMPERATURE_GPIO_PORT         GPIOA
#define ADC1_CHANNEL_TEMPERATURE_PIN               GPIO_PIN_5

/* Definition of ADCx NVIC resources */
#define ADC1_IRQn                       ADC1_COMP_IRQn
#define ADC1_IRQHandler                 ADC1_COMP_IRQHandler


#define FASTER_COMMAND_PARSER



#endif /* CONFIGURATION_H_ */
