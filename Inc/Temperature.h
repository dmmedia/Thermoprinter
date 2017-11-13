/*
 * Temperature.h
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include "Conditionals.h"
#include "stm32l0xx_hal.h"

#define HOTEND_LOOP() for (int8_t e = 0; e < HOTENDS; e++)

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


#define OVERSAMPLENR 16

// User-defined table 1
// Dummy Thermistor table.. It will ALWAYS read a fixed value.
#ifndef DUMMY_THERMISTOR_998_VALUE
  #define DUMMY_THERMISTOR_998_VALUE 25
#endif

const short temptable_998[][2] = {
  {    1 * OVERSAMPLENR, DUMMY_THERMISTOR_998_VALUE },
  { 1023 * OVERSAMPLENR, DUMMY_THERMISTOR_998_VALUE }
};

// Set the high and low raw values for the heaters
// For thermistors the highest temperature results in the lowest ADC value
// For thermocouples the highest temperature results in the highest ADC value
#ifndef PRINTHEAD_RAW_HI_TEMP
  #ifdef PRINTHEAD_USES_THERMISTOR
    #define PRINTHEAD_RAW_HI_TEMP 0
    #define PRINTHEAD_RAW_LO_TEMP 16383
  #else
    #define PRINTHEAD_RAW_HI_TEMP 16383
    #define PRINTHEAD_RAW_LO_TEMP 0
  #endif
#endif

#define _TT_NAME(_N) temptable_ ## _N
#define TT_NAME(_N) _TT_NAME(_N)

#ifdef THERMISTORHEATER_0
  #define PRINTHEAD_TEMPTABLE TT_NAME(THERMISTORHEATER_0)
  #define PRINTHEAD_TEMPTABLE_LEN COUNT(PRINTHEAD_TEMPTABLE)
#elif defined(PRINTHEAD_USES_THERMISTOR)
  #error "No heater 0 thermistor table specified"
#else
  #define PRINTHEAD_TEMPTABLE NULL
  #define PRINTHEAD_TEMPTABLE_LEN 0
#endif

/**
 * States for ADC reading in the ISR
 */
enum ADCSensorState {
  #if HAS_TEMP_0
    PrepareTemp_0,
    MeasureTemp_0,
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    Prepare_FILWIDTH,
    Measure_FILWIDTH,
  #endif
  SensorsReady, // Temperatures ready. Delay the next round of readings to let ADC pins settle.
  StartupDelay  // Startup, delay initial temp reading a tiny bit so the hardware can settle
};

// Minimum number of Temperature::ISR loops between sensor readings.
// Multiplied by 16 (OVERSAMPLENR) to obtain the total time to
// get all oversampled sensor readings
#define MIN_ADC_ISR_LOOPS 10

class Temperature {
public:
	Temperature() {};

    void init();

    /**
     * Static (class) methods
     */
    static float analog2temp(int raw);

    /**
     * Called from the Temperature ISR
     */
    static void isr();

    static volatile bool in_temp_isr;

    static int16_t current_temperature_raw;

private:
    // Init min and max temp with extreme values to prevent false errors during startup
    static int16_t minttemp_raw,
                   maxttemp_raw,
                   minttemp,
                   maxttemp;

    static volatile bool temp_meas_ready;

    static void set_current_temp_raw();

    static uint16_t raw_temp_value;

    static void _temp_error(const char * const serial_msg, const char * const lcd_msg);
    static void max_temp_error();
    static void min_temp_error();

    ADC_HandleTypeDef    AdcHandle;

    static void ADC_Config(void);

    /**
     * Switch off all heaters, set all target temperatures to 0
     */
    static void disable_all_heaters();

};

#endif /* TEMPERATURE_H_ */
