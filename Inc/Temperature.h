/*
 * Temperature.h
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

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

// Set the high and low raw values for the print head
// For thermistors the highest temperature results in the lowest ADC value
#ifndef PRINTHEAD_RAW_HI_TEMP
  #define PRINTHEAD_RAW_HI_TEMP 0
  #define PRINTHEAD_RAW_LO_TEMP 16383
#endif

#define _TT_NAME(_N) temptable_ ## _N
#define TT_NAME(_N) _TT_NAME(_N)

#if defined(THERMISTORHEATER_0)
  #define PRINTHEAD_TEMPTABLE TT_NAME(THERMISTORHEATER_0)
  #define PRINTHEAD_TEMPTABLE_LEN COUNT(PRINTHEAD_TEMPTABLE)
#elif defined(PRINTHEAD_USES_THERMISTOR)
  #error "No heater 0 thermistor table specified"
#else
  #define PRINTHEAD_TEMPTABLE NULL
  #define PRINTHEAD_TEMPTABLE_LEN 0
#endif

// Set the high and low raw values for the battery
#ifndef BATTERY_RAW_HI_VOLT
  #define BATTERY_RAW_HI_VOLT 16383
  #define BATTERY_RAW_LO_VOLT 0
#endif

#define BATTERY_VOLTTABLE TT_NAME(THERMISTORHEATER_0) // TODO: fixme
#define BATTERY_VOLTTABLE_LEN COUNT(BATTERY_VOLTTABLE)

/**
 * States for ADC reading in the ISR
 */
enum ADCSensorState {
  PrepareSensors,
  MeasureSensors,
  SensorsReady, // Temperatures ready. Delay the next round of readings to let ADC pins settle.
  StartupDelay  // Startup, delay initial temp reading a tiny bit so the hardware can settle
};

// Minimum number of Temperature::ISR loops between sensor readings.
// Multiplied by 16 (OVERSAMPLENR) to obtain the total time to
// get all oversampled sensor readings
#define MIN_ADC_ISR_LOOPS 10

extern TIM_HandleTypeDef htim2;

/* ADC parameters */
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)    2)    /* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */

extern ADC_HandleTypeDef AdcHandle;

class Temperature {
public:
	Temperature() {};

    void init();

    /**
     * Static (class) methods
     */
    static float analog2temp(int raw);
    static float analog2volt(int raw);

    /**
     * Called from the Temperature ISR
     */
    static void isr();

    static volatile bool in_temp_isr;

    static int16_t current_temperature_raw, current_voltage_raw;

    /**
     * Switch off all heaters, set all target temperatures to 0
     */
    static void disable_all_heaters();

private:
    // Init min and max temp with extreme values to prevent false errors during startup
    static int16_t minttemp_raw,
                   maxttemp_raw,
                   minttemp,
                   maxttemp;

    // Init min and max volt with extreme values to prevent false errors during startup
    static int16_t mintvolt_raw,
                   maxtvolt_raw,
                   mintvolt,
                   maxtvolt;

    static volatile bool sens_meas_ready;

    static void set_current_sens_raw();

    static uint16_t raw_temp_value, raw_volt_value;

    static void _sens_error(const char * const serial_msg, const char * const lcd_msg);
    static void max_sens_error();
    static void min_sens_error();

    void ADC_Config(void);

};

extern Temperature thermalManager;

#endif /* TEMPERATURE_H_ */
