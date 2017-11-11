/*
 * Temperature.h
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include "Conditionals.h"
#include "main.h"
#include "stm32l0xx_hal.h"

#define HOTEND_LOOP() for (int8_t e = 0; e < HOTENDS; e++)

#define ADCx                            ADC1

/* Definition of ADCx channels */
#define ADCx_CHANNEL_VOLTAGE                   ADC_CHANNEL_8
#define ADCx_CHANNEL_TEMPERATURE               ADC_CHANNEL_5

/* Definition of ADCx channels pins */
//#define ADCx_CHANNEL_VOLTAGE_GPIO_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define ADCx_CHANNEL_VOLTAGE_GPIO_PORT         GPIOB
#define ADCx_CHANNEL_VOLTAGE_PIN               GPIO_PIN_0

//#define ADCx_CHANNEL_TEMPERATURE_GPIO_CLK_ENABLE() __GPIOA_CLK_ENABLE()
#define ADCx_CHANNEL_TEMPERATURE_GPIO_PORT         GPIOA
#define ADCx_CHANNEL_TEMPERATURE_PIN               GPIO_PIN_5


// Set the high and low raw values for the heaters
// For thermistors the highest temperature results in the lowest ADC value
// For thermocouples the highest temperature results in the highest ADC value
#ifndef HEATER_0_RAW_HI_TEMP
  #ifdef HEATER_0_USES_THERMISTOR
    #define HEATER_0_RAW_HI_TEMP 0
    #define HEATER_0_RAW_LO_TEMP 16383
  #else
    #define HEATER_0_RAW_HI_TEMP 16383
    #define HEATER_0_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_1_RAW_HI_TEMP
  #ifdef HEATER_1_USES_THERMISTOR
    #define HEATER_1_RAW_HI_TEMP 0
    #define HEATER_1_RAW_LO_TEMP 16383
  #else
    #define HEATER_1_RAW_HI_TEMP 16383
    #define HEATER_1_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_2_RAW_HI_TEMP
  #ifdef HEATER_2_USES_THERMISTOR
    #define HEATER_2_RAW_HI_TEMP 0
    #define HEATER_2_RAW_LO_TEMP 16383
  #else
    #define HEATER_2_RAW_HI_TEMP 16383
    #define HEATER_2_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_3_RAW_HI_TEMP
  #ifdef HEATER_3_USES_THERMISTOR
    #define HEATER_3_RAW_HI_TEMP 0
    #define HEATER_3_RAW_LO_TEMP 16383
  #else
    #define HEATER_3_RAW_HI_TEMP 16383
    #define HEATER_3_RAW_LO_TEMP 0
  #endif
#endif
#ifndef HEATER_4_RAW_HI_TEMP
  #ifdef HEATER_4_USES_THERMISTOR
    #define HEATER_4_RAW_HI_TEMP 0
    #define HEATER_4_RAW_LO_TEMP 16383
  #else
    #define HEATER_4_RAW_HI_TEMP 16383
    #define HEATER_4_RAW_LO_TEMP 0
  #endif
#endif

class Temperature {
public:
	Temperature() {};

    void init();

    static volatile bool in_temp_isr;

private:
    // Init min and max temp with extreme values to prevent false errors during startup
    static int16_t minttemp_raw[HOTENDS],
                   maxttemp_raw[HOTENDS],
                   minttemp[HOTENDS],
                   maxttemp[HOTENDS];

    ADC_HandleTypeDef    AdcHandle;

    static void ADC_Config(void);
};

#endif /* TEMPERATURE_H_ */
