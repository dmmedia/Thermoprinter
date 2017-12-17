/*
 * Temperature.h
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#ifndef TEMPERATURE_H_
#define TEMPERATURE_H_

#include "typedefs.h"

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

/**
  * @brief  ADC group regular oversampling structure definition
  */
typedef struct
{
  uint32_t Ratio;                         /*!< Configures the oversampling ratio.
                                               This parameter can be a value of @ref ADC_Oversampling_Ratio */

  uint32_t RightBitShift;                 /*!< Configures the division coefficient for the Oversampler.
                                               This parameter can be a value of @ref ADC_Right_Bit_Shift */

  uint32_t TriggeredMode;                 /*!< Selects the regular triggered oversampling mode.
                                               This parameter can be a value of @ref ADC_Triggered_Oversampling_Mode */
} ADC_OversamplingTypeDef;

/**
  * @brief  Structure definition of ADC instance and ADC group regular.
  * @note   Parameters of this structure are shared within 2 scopes:
  *          - Scope entire ADC (differentiation done for compatibility with some other STM32 series featuring ADC groups regular and injected): ClockPrescaler, Resolution, DataAlign,
  *            ScanConvMode, EOCSelection, LowPowerAutoWait.
  *          - Scope ADC group regular: ContinuousConvMode, NbrOfConversion, DiscontinuousConvMode,
  *            ExternalTrigConv, ExternalTrigConvEdge, DMAContinuousRequests, Overrun, OversamplingMode, Oversampling.
  * @note   The setting of these parameters by function HAL_ADC_Init() is conditioned to ADC state.
  *         ADC state can be either:
  *          - For all parameters: ADC disabled
  *          - For all parameters except 'ClockPrescaler' and 'Resolution': ADC enabled without conversion on going on group regular.
  *         If ADC is not in the appropriate state to modify some parameters, these parameters setting is bypassed
  *         without error reporting (as it can be the expected behavior in case of intended action to update another parameter
  *         (which fulfills the ADC state condition) on the fly).
  */
typedef struct
{
  uint32_t ClockPrescaler;        /*!< Select ADC clock source (synchronous clock derived from APB clock or asynchronous clock derived from ADC dedicated HSI RC oscillator) and clock prescaler.
                                       This parameter can be a value of @ref ADC_ClockPrescaler.
                                       Note: In case of synchronous clock mode based on HCLK/1, the configuration must be enabled only
                                             if the system clock has a 50% duty clock cycle (APB prescaler configured inside RCC
                                             must be bypassed and PCLK clock must have 50% duty cycle). Refer to reference manual for details.
                                       Note: In case of usage of the ADC dedicated HSI RC oscillator, it must be preliminarily enabled at RCC top level.
                                       Note: This parameter can be modified only if the ADC is disabled. */

  uint32_t Resolution;            /*!< Configure the ADC resolution.
                                       This parameter can be a value of @ref ADC_Resolution */

  uint32_t DataAlign;             /*!< Specify ADC data alignment in conversion data register (right or left).
                                       Refer to reference manual for alignments formats versus resolutions.
                                       This parameter can be a value of @ref ADC_Data_align */

  uint32_t ScanConvMode;          /*!< Configure the sequencer of regular group.
                                       This parameter can be associated to parameter 'DiscontinuousConvMode' to have main sequence subdivided in successive parts.
                                       Sequencer is automatically enabled if several channels are set (sequencer cannot be disabled, as it can be the case on other STM32 devices):
                                       If only 1 channel is set: Conversion is performed in single mode.
                                       If several channels are set:  Conversions are performed in sequence mode (ranks defined by each channel number: channel 0 fixed on rank 0, channel 1 fixed on rank1, ...).
                                                                     Scan direction can be set to forward (from channel 0 to channel 18) or backward (from channel 18 to channel 0).
                                       This parameter can be a value of @ref ADC_Scan_mode */

  uint32_t EOCSelection;          /*!< Specify which EOC (End Of Conversion) flag is used for conversion by polling and interruption: end of unitary conversion or end of sequence conversions.
                                       This parameter can be a value of @ref ADC_EOCSelection. */

  uint32_t LowPowerAutoWait;      /*!< Select the dynamic low power Auto Delay: new conversion start only when the previous
                                       conversion (for ADC group regular) has been retrieved by user software,
                                       using function HAL_ADC_GetValue().
                                       This feature automatically adapts the frequency of ADC conversions triggers to the speed of the system that reads the data. Moreover, this avoids risk of overrun
                                       for low frequency applications.
                                       This parameter can be set to ENABLE or DISABLE.
                                       Note: Do not use with interruption or DMA (HAL_ADC_Start_IT(), HAL_ADC_Start_DMA()) since they clear immediately the EOC flag
                                             to free the IRQ vector sequencer.
                                             Do use with polling: 1. Start conversion with HAL_ADC_Start(), 2. Later on, when ADC conversion data is needed:
                                             use HAL_ADC_PollForConversion() to ensure that conversion is completed and HAL_ADC_GetValue() to retrieve conversion result and trig another conversion start. */

  uint32_t LowPowerAutoPowerOff;  /*!< Select the auto-off mode: the ADC automatically powers-off after a conversion and automatically wakes-up when a new conversion is triggered (with startup time between trigger and start of sampling).
                                       This feature can be combined with automatic wait mode (parameter 'LowPowerAutoWait').
                                       This parameter can be set to ENABLE or DISABLE.
                                       Note: If enabled, this feature also turns off the ADC dedicated 14 MHz RC oscillator (HSI14) */

  uint32_t ContinuousConvMode;    /*!< Specify whether the conversion is performed in single mode (one conversion) or continuous mode for ADC group regular,
                                       after the first ADC conversion start trigger occurred (software start or external trigger).
                                       This parameter can be set to ENABLE or DISABLE. */

  uint32_t DiscontinuousConvMode; /*!< Specify whether the conversions sequence of ADC group regular is performed in Complete-sequence/Discontinuous-sequence
                                       (main sequence subdivided in successive parts).
                                       Discontinuous mode is used only if sequencer is enabled (parameter 'ScanConvMode'). If sequencer is disabled, this parameter is discarded.
                                       Discontinuous mode can be enabled only if continuous mode is disabled. If continuous mode is enabled, this parameter setting is discarded.
                                       This parameter can be set to ENABLE or DISABLE.
                                       Note: On this STM32 serie, ADC group regular number of discontinuous ranks increment is fixed to one-by-one. */

  uint32_t ExternalTrigConv;      /*!< Select the external event source used to trigger ADC group regular conversion start.
                                       If set to ADC_SOFTWARE_START, external triggers are disabled and software trigger is used instead.
                                       This parameter can be a value of @ref ADC_regular_external_trigger_source.
                                       Caution: external trigger source is common to all ADC instances. */

  uint32_t ExternalTrigConvEdge;  /*!< Select the external event edge used to trigger ADC group regular conversion start.
                                       If trigger source is set to ADC_SOFTWARE_START, this parameter is discarded.
                                       This parameter can be a value of @ref ADC_regular_external_trigger_edge */

  uint32_t DMAContinuousRequests; /*!< Specify whether the DMA requests are performed in one shot mode (DMA transfer stops when number of conversions is reached)
                                       or in continuous mode (DMA transfer unlimited, whatever number of conversions).
                                       This parameter can be set to ENABLE or DISABLE.
                                       Note: In continuous mode, DMA must be configured in circular mode. Otherwise an overrun will be triggered when DMA buffer maximum pointer is reached. */

  uint32_t Overrun;               /*!< Select the behavior in case of overrun: data overwritten or preserved (default).
                                       This parameter can be a value of @ref ADC_Overrun.
                                       Note: In case of overrun set to data preserved and usage with programming model with interruption (HAL_Start_IT()): ADC IRQ handler has to clear
                                       end of conversion flags, this induces the release of the preserved data. If needed, this data can be saved in function
                                       HAL_ADC_ConvCpltCallback(), placed in user program code (called before end of conversion flags clear).
                                       Note: Error reporting with respect to the conversion mode:
                                             - Usage with ADC conversion by polling for event or interruption: Error is reported only if overrun is set to data preserved. If overrun is set to data
                                               overwritten, user can willingly not read all the converted data, this is not considered as an erroneous case.
                                             - Usage with ADC conversion by DMA: Error is reported whatever overrun setting (DMA is expected to process all data from data register). */

  uint32_t LowPowerFrequencyMode; /*!< When selecting an analog ADC clock frequency lower than 2.8MHz,
                                       it is mandatory to first enable the Low Frequency Mode.
                                       This parameter can be set to ENABLE or DISABLE.
                                       Note: This parameter can be modified only if there is no conversion is ongoing. */


  uint32_t SamplingTime;                 /*!< The sample time common to all channels.
                                              Unit: ADC clock cycles
                                              This parameter can be a value of @ref ADC_sampling_times
                                              Note: This parameter can be modified only if there is no conversion ongoing. */

  uint32_t OversamplingMode;              /*!< Specify whether the oversampling feature is enabled or disabled.
                                               This parameter can be set to ENABLE or DISABLE.
                                               Note: This parameter can be modified only if there is no conversion is ongoing on ADC group regular. */


  ADC_OversamplingTypeDef  Oversample;   /*!< Specify the Oversampling parameters
                                              Caution: this setting overwrites the previous oversampling configuration if oversampling is already enabled. */
} ADC_InitTypeDef;

/**
  * @brief  ADC handle Structure definition
  */
typedef struct
{
  ADC_TypeDef                   *Instance;              /*!< Register base address */

  ADC_InitTypeDef               Init;                   /*!< ADC required parameters */

  DMA_HandleTypeDef             *DMA_Handle;            /*!< Pointer DMA Handler */

  HandleLockTypeDef               Lock;                   /*!< ADC locking object */

  __IO uint32_t                 State;                  /*!< ADC communication state (bitmap of ADC states) */

  __IO uint32_t                 ErrorCode;              /*!< ADC Error code */
} ADC_HandleTypeDef;

/** @defgroup ADC_channels ADC_Channels
  * @{
  */
#define ADC_CHANNEL_0           ((uint32_t)(ADC_CHSELR_CHSEL0))
#define ADC_CHANNEL_1           ((uint32_t)(ADC_CHSELR_CHSEL1) | ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_2           ((uint32_t)(ADC_CHSELR_CHSEL2) | ADC_CFGR1_AWDCH_1)
#define ADC_CHANNEL_3           ((uint32_t)(ADC_CHSELR_CHSEL3)| ADC_CFGR1_AWDCH_1 | ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_4           ((uint32_t)(ADC_CHSELR_CHSEL4)| ADC_CFGR1_AWDCH_2)
#define ADC_CHANNEL_5           ((uint32_t)(ADC_CHSELR_CHSEL5)| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_6           ((uint32_t)(ADC_CHSELR_CHSEL6)| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_1)
#define ADC_CHANNEL_7           ((uint32_t)(ADC_CHSELR_CHSEL7)| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_1 | ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_8           ((uint32_t)(ADC_CHSELR_CHSEL8)| ADC_CFGR1_AWDCH_3)
#define ADC_CHANNEL_9           ((uint32_t)(ADC_CHSELR_CHSEL9)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_10          ((uint32_t)(ADC_CHSELR_CHSEL10)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_1)
#define ADC_CHANNEL_11          ((uint32_t)(ADC_CHSELR_CHSEL11)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_1| ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_12          ((uint32_t)(ADC_CHSELR_CHSEL12)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_2)
#define ADC_CHANNEL_13          ((uint32_t)(ADC_CHSELR_CHSEL13)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_14          ((uint32_t)(ADC_CHSELR_CHSEL14)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_1)
#define ADC_CHANNEL_15          ((uint32_t)(ADC_CHSELR_CHSEL15)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_1| ADC_CFGR1_AWDCH_0)
#if defined (STM32L053xx) || defined (STM32L063xx) || defined (STM32L073xx) || defined (STM32L083xx)
#define ADC_CHANNEL_16          ((uint32_t)(ADC_CHSELR_CHSEL16)| ADC_CFGR1_AWDCH_4)
#endif
#define ADC_CHANNEL_17          ((uint32_t)(ADC_CHSELR_CHSEL17)| ADC_CFGR1_AWDCH_4| ADC_CFGR1_AWDCH_0)
#define ADC_CHANNEL_18          ((uint32_t)(ADC_CHSELR_CHSEL18)| ADC_CFGR1_AWDCH_4| ADC_CFGR1_AWDCH_1)

/* Internal channels */
#if defined (STM32L053xx) || defined (STM32L063xx) || defined (STM32L073xx) || defined (STM32L083xx)
#define ADC_CHANNEL_VLCD         ADC_CHANNEL_16
#endif
#define ADC_CHANNEL_VREFINT      ADC_CHANNEL_17
#define ADC_CHANNEL_TEMPSENSOR   ADC_CHANNEL_18

/* Definition of ADCx clock resources */
#define ADC1_CLK_ENABLE()               SET_BIT(RCC->APB2ENR, (RCC_APB2ENR_ADC1EN))

#define ADC1_FORCE_RESET()              SET_BIT(RCC->APB2RSTR, (RCC_APB2RSTR_ADC1RST))
#define ADC1_RELEASE_RESET()            CLEAR_BIT(RCC->APB2RSTR, (RCC_APB2RSTR_ADC1RST))

void ADC1_IRQHandler(void);

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
