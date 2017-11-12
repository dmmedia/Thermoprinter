/*
 * Temperature.cpp
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#include <Temperature.h>
#include "macros.h"

Temperature thermalManager;

// Init min and max temp with extreme values to prevent false errors during startup
int16_t Temperature::minttemp_raw = PRINTHEAD_RAW_LO_TEMP,
        Temperature::maxttemp_raw = PRINTHEAD_RAW_HI_TEMP,
        Temperature::minttemp = 0,
        Temperature::maxttemp = 16383;

static void* heater_ttbl_map = (void*)PRINTHEAD_TEMPTABLE;
static uint8_t heater_ttbllen_map = PRINTHEAD_TEMPTABLE_LEN;

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_heater()
 */
void Temperature::init() {

  ADC_Config();

  /* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
    Error_Handler();
  }

  // Use timer TIM2 for temperature measurement
  __HAL_TIM_SET_AUTORELOAD(&htim2, 128);
  NVIC_EnableIRQ(TIM2_IRQn);

  // Wait for temperature measurement to settle
  HAL_Delay(250);

  #define TEMP_MIN_ROUTINE() \
    minttemp = PRINTHEAD_MINTEMP; \
    while (analog2temp(minttemp_raw) < PRINTHEAD_MINTEMP) { \
      if (PRINTHEAD_RAW_LO_TEMP < PRINTHEAD_RAW_HI_TEMP) \
        minttemp_raw += OVERSAMPLENR; \
      else \
        minttemp_raw -= OVERSAMPLENR; \
    }
  #define TEMP_MAX_ROUTINE() \
    maxttemp = PRINTHEAD_MAXTEMP; \
    while (analog2temp(maxttemp_raw) > PRINTHEAD_MAXTEMP) { \
      if (PRINTHEAD_RAW_LO_TEMP < PRINTHEAD_RAW_HI_TEMP) \
        maxttemp_raw -= OVERSAMPLENR; \
      else \
        maxttemp_raw += OVERSAMPLENR; \
    }

  #ifdef PRINTHEAD_MINTEMP
    TEMP_MIN_ROUTINE();
  #endif
  #ifdef PRINTHEAD_MAXTEMP
    TEMP_MAX_ROUTINE();
  #endif
}

/**
  * @brief  ADC configuration
  * @param  None
  * @retval None
  */
static void Temperature::ADC_Config(void)
{
  ADC_ChannelConfTypeDef   sConfig;

  /* Configuration of AdcHandle init structure: ADC parameters and regular group */
  AdcHandle.Instance = ADC1;

  if (HAL_ADC_DeInit(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }

  AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
  AdcHandle.Init.Resolution            = ADC_RESOLUTION12b;
  AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;    /* Sequencer will convert the number of channels configured below, successively from the lowest to the highest channel number */
  AdcHandle.Init.EOCSelection          = EOC_SINGLE_CONV;
  AdcHandle.Init.LowPowerAutoWait      = DISABLE;
  AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
  AdcHandle.Init.ContinuousConvMode    = DISABLE;                       /* Continuous mode disabled to have only 1 rank converted at each conversion trig, and because discontinuous mode is enabled */
  AdcHandle.Init.DiscontinuousConvMode = ENABLE;                        /* Sequencer of regular group will convert the sequence in several sub-divided sequences */
  AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIG2_T2_TRGO;     /* Parameter discarded because trig of conversion without external event */
  AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIG_EDGE_NONE;    /* Trig of conversion without external event: start done manually by software */
  AdcHandle.Init.DMAContinuousRequests = ENABLE;                        /* ADC-DMA continuous requests to match with DMA configured in circular mode */
  AdcHandle.Init.Overrun               = OVR_DATA_OVERWRITTEN;
  AdcHandle.Init.LowPowerFrequencyMode = DISABLE;
  /* Note: Set long sampling time due to internal channels (VrefInt,          */
  /*       temperature sensor) constraints. Refer to device datasheet for     */
  /*       min/typ/max values.                                                */
  AdcHandle.Init.SamplingTime          = ADC_SAMPLETIME_79CYCLES_5;
  AdcHandle.Init.OversamplingMode      = DISABLE;


  if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
  {
    /* ADC initialization error */
    Error_Handler();
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 1 */
  /* Note: Considering IT occurring after each ADC conversion (IT by DMA end  */
  /*       of transfer), select sampling time and ADC clock with sufficient   */
  /*       duration to not create an overhead situation in IRQHandler.        */
  sConfig.Channel      = ADC1_CHANNEL_TEMPERATURE;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

  /* Configuration of channel on ADCx regular group on sequencer rank 2 */
  /* Replicate previous rank settings, change only channel */
  /* Note: On STM32L0xx, rank is defined by channel number. ADC Channel         */
  /*       ADC_CHANNEL_VREFINT is on ADC channel 17, there is 1 other         */
  /*       channel enabled with lower channel number. Therefore,              */
  /*       ADC_CHANNEL_VREFINT will be converted by the sequencer as the      */
  /*       2nd rank.                                                          */
  sConfig.Channel      = ADC1_CHANNEL_VOLTAGE;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

}

/**
 * This ISR uses the compare method so it runs at the base
 * frequency (16 MHz / 64 / 256 = 976.5625 Hz), but at the CNT set
 * in ARR above (128 or halfway between OVFs).
 *
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - Check new temperature values for MIN/MAX errors (kill on error)
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 */
void TIM2_IRQHandler(void)
{
	Temperature::isr();
}

volatile bool Temperature::in_temp_isr = false;

void Temperature::isr() {
  // The stepper ISR can interrupt this ISR. When it does it re-enables this ISR
  // at the end of its run, potentially causing re-entry. This flag prevents it.
  if (in_temp_isr) return;
  in_temp_isr = true;

  // Allow UART and stepper ISRs
  NVIC_DisableIRQ(TIM2_IRQn);
  sei();

  static int8_t temp_count = -1;
  static ADCSensorState adc_sensor_state = StartupDelay;
  static uint8_t pwm_count = _BV(SOFT_PWM_SCALE);
  // avoid multiple loads of pwm_count
  uint8_t pwm_count_tmp = pwm_count;

  #define ISR_STATICS(n) static uint8_t soft_pwm_count_ ## n = 0

  // Statics per heater
  ISR_STATICS(0);

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    static unsigned long raw_filwidth_value = 0;
  #endif

  /**
   * One sensor is sampled on every other call of the ISR.
   * Each sensor is read 16 (OVERSAMPLENR) times, taking the average.
   *
   * On each Prepare pass, ADC is started for a sensor pin.
   * On the next pass, the ADC value is read and accumulated.
   *
   * This gives each ADC 0.9765ms to charge up.
   */

  #define SET_ADMUX_ADCSRA(pin) ADMUX = _BV(REFS0) | (pin & 0x07); SBI(ADCSRA, ADSC)
  #ifdef MUX5
    #define START_ADC(pin) if (pin > 7) ADCSRB = _BV(MUX5); else ADCSRB = 0; SET_ADMUX_ADCSRA(pin)
  #else
    #define START_ADC(pin) ADCSRB = 0; SET_ADMUX_ADCSRA(pin)
  #endif

  switch (adc_sensor_state) {

    case SensorsReady: {
      // All sensors have been read. Stay in this state for a few
      // ISRs to save on calls to temp update/checking code below.
      constexpr int8_t extra_loops = MIN_ADC_ISR_LOOPS - (int8_t)SensorsReady;
      static uint8_t delay_count = 0;
      if (extra_loops > 0) {
        if (delay_count == 0) delay_count = extra_loops;   // Init this delay
        if (--delay_count)                                 // While delaying...
          adc_sensor_state = (ADCSensorState)(int(SensorsReady) - 1); // retain this state (else, next state will be 0)
        break;
      }
      else
        adc_sensor_state = (ADCSensorState)0; // Fall-through to start first sensor now
    }

    #if HAS_TEMP_0
      case PrepareTemp_0:
        START_ADC(TEMP_0_PIN);
        break;
      case MeasureTemp_0:
        raw_temp_value[0] += ADC;
        break;
    #endif

    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      case Prepare_FILWIDTH:
        START_ADC(FILWIDTH_PIN);
      break;
      case Measure_FILWIDTH:
        if (ADC > 102) { // Make sure ADC is reading > 0.5 volts, otherwise don't read.
          raw_filwidth_value -= (raw_filwidth_value >> 7); // Subtract 1/128th of the raw_filwidth_value
          raw_filwidth_value += ((unsigned long)ADC << 7); // Add new ADC reading, scaled by 128
        }
      break;
    #endif

    case StartupDelay: break;

  } // switch(adc_sensor_state)

  if (!adc_sensor_state && ++temp_count >= OVERSAMPLENR) { // 10 * 16 * 1/(16000000/64/256)  = 164ms.

    temp_count = 0;

    // Update the raw values if they've been read. Else we could be updating them during reading.
    if (!temp_meas_ready) set_current_temp_raw();

    // Filament Sensor - can be read any time since IIR filtering is used
    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      current_raw_filwidth = raw_filwidth_value >> 10;  // Divide to get to 0-16384 range since we used 1/128 IIR filter approach
    #endif

    ZERO(raw_temp_value);
    raw_temp_bed_value = 0;

    #define TEMPDIR(N) ((PRINTHEAD_RAW_LO_TEMP) > (PRINTHEAD_RAW_HI_TEMP) ? -1 : 1)

    int constexpr temp_dir = TEMPDIR(0);

    const int16_t tdir = temp_dir, rawtemp = current_temperature_raw * tdir;
    if (rawtemp > maxttemp_raw * tdir && target_temperature > 0) max_temp_error();
    if (rawtemp < minttemp_raw * tdir && !is_preheating() && target_temperature > 0) {
      #ifdef MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED
        if (++consecutive_low_temperature_error >= MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
      #endif
          min_temp_error();
    }
    #ifdef MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED
      else
        consecutive_low_temperature_error = 0;
    #endif

  } // temp_count >= OVERSAMPLENR

  // Go to the next state, up to SensorsReady
  adc_sensor_state = (ADCSensorState)((int(adc_sensor_state) + 1) % int(StartupDelay));

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)

    extern volatile uint8_t e_hit;

    if (e_hit && endstops.enabled) {
      endstops.update();  // call endstop update routine
      e_hit--;
    }
  #endif

  cli();
  in_temp_isr = false;
  NVIC_EnableIRQ(TIM2_IRQn); //re-enable Temperature ISR
}

// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
float Temperature::analog2temp(int raw) {
  if (heater_ttbl_map != NULL) {
    float celsius = 0;
    uint8_t i;
    short(*tt)[][2] = (short(*)[][2])(heater_ttbl_map);

    for (i = 1; i < heater_ttbllen_map; i++) {
      if ((short)((*tt)[i][0]) > raw) {
        celsius = (short)((*tt)[i - 1][1]) +
                  (raw - (short)((*tt)[i - 1][0])) *
                  (float)((short)((*tt)[i][1]) - (short)((*tt)[i - 1][1])) /
                  (float)((short)((*tt)[i][0]) - (short)((*tt)[i - 1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == heater_ttbllen_map) celsius = (short)((*tt)[i - 1][1]);

    return celsius;
  }
  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR) * (TEMP_SENSOR_AD595_GAIN)) + TEMP_SENSOR_AD595_OFFSET;
}

