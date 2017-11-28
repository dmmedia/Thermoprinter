/*
 * Temperature.cpp
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#include "stm32l0xx_hal.h"
#include "Configuration.h"
#include "Conditionals.h"
#include "macros.h"
#include "Temperature.h"
#include "SREGEmulation.h"
#include "Endstops.h"
#include "Stopwatch.h"

Temperature thermalManager;

TIM_HandleTypeDef htim2;

/* Variable containing ADC conversions results */
__IO uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

/* Variable to report ADC sequencer status */
uint8_t         ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */

// Init min and max temp with extreme values to prevent false errors during startup
int16_t Temperature::minttemp_raw = PRINTHEAD_RAW_LO_TEMP,
        Temperature::maxttemp_raw = PRINTHEAD_RAW_HI_TEMP,
        Temperature::minttemp = 0,
        Temperature::maxttemp = 16383;

// Init min and max volt with extreme values to prevent false errors during startup
int16_t Temperature::mintvolt_raw = BATTERY_RAW_LO_VOLT,
        Temperature::maxtvolt_raw = BATTERY_RAW_HI_VOLT,
        Temperature::mintvolt = 0,
        Temperature::maxtvolt = 16383;

static void* heater_ttbl_map = (void*)PRINTHEAD_TEMPTABLE;
static uint8_t heater_ttbllen_map = PRINTHEAD_TEMPTABLE_LEN;

static void* battery_ttbl_map = (void*)BATTERY_VOLTTABLE;
static uint8_t battery_ttbllen_map = BATTERY_VOLTTABLE_LEN;

volatile bool Temperature::sens_meas_ready = false;

uint16_t Temperature::raw_temp_value = 0;
uint16_t Temperature::raw_volt_value = 0;

int16_t Temperature::current_temperature_raw = 0, current_voltage_raw = 0;

ADC_HandleTypeDef AdcHandle;

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




  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TIM_Base_InitTypeDef tim2_Init;
  tim2_Init.Prescaler = 64;
  tim2_Init.CounterMode = TIM_COUNTERMODE_UP;
  // Init Stepper ISR to 4 us Hz
  tim2_Init.Period = 256;
  tim2_Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim2.Instance = TIM6;
  htim2.Init = tim2_Init;
  htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;

  if (HAL_ERROR == HAL_TIM_Base_Init(&htim2)) {
	  Error_Handler();
  }

  // Set the timer pre-scaler
  // Generally we use a divider of 64, resulting in a 250KHz timer
  // frequency on a 16MHz MCU.
  //TIM2->PSC = 64; // 1/64 prescaler

  // Init Stepper ISR to 4 us
  //TIM2->ARR = 256;

  // Enable update interrupts
  //TIM2->DIER |= TIM_DIER_UIE;
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  //TIM2->CNT = 0;
  __HAL_TIM_SET_COUNTER(&htim2, 0);




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

  #define VOLT_MIN_ROUTINE() \
    mintvolt = BATTERY_MINVOLT; \
    while (analog2volt(mintvolt_raw) < BATTERY_MINVOLT) { \
      if (BATTERY_RAW_LO_VOLT < BATTERY_RAW_HI_VOLT) \
        mintvolt_raw += OVERSAMPLENR; \
      else \
        mintvolt_raw -= OVERSAMPLENR; \
    }
  #define VOLT_MAX_ROUTINE() \
    maxttemp = BATTERY_MAXVOLT; \
    while (analog2volt(maxtvolt_raw) > BATTERY_MAXVOLT) { \
      if (BATTERY_RAW_LO_VOLT < BATTERY_RAW_HI_VOLT) \
        maxtvolt_raw -= OVERSAMPLENR; \
      else \
        maxtvolt_raw += OVERSAMPLENR; \
    }

  #ifdef BATTERY_MINVOLT
    VOLT_MIN_ROUTINE();
  #endif
  #ifdef BATTERY_MAXVOLT
    VOLT_MAX_ROUTINE();
  #endif
}

/**
  * @brief  ADC configuration
  * @param  None
  * @retval None
  */
void Temperature::ADC_Config(void)
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
  AdcHandle.Init.Resolution            = ADC_RESOLUTION10b;
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

  /*## Start ADC conversions #################################################*/

  /* Start ADC conversion on regular group with transfer by DMA */
  if (HAL_ADC_Start_DMA(&AdcHandle,
                        (uint32_t *)aADCxConvertedValues,
                        ADCCONVERTEDVALUES_BUFFER_SIZE
                       ) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /* Run the ADC calibration in single-ended mode */
  if (HAL_ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) != HAL_OK)
  {
    /* Calibration Error */
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

  /**
   * One sensor is sampled on every other call of the ISR.
   * Each sensor is read 16 (OVERSAMPLENR) times, taking the average.
   *
   * On each Prepare pass, ADC is started for a sensor pin.
   * On the next pass, the ADC value is read and accumulated.
   *
   * This gives each ADC 0.9765ms to charge up.
   */

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
      }
      else
        adc_sensor_state = (ADCSensorState)0; // Fall-through to start first sensor now
      break;
    }

    case PrepareSensors:
      /* Start ADC conversion */
      /* Since sequencer is enabled in discontinuous mode, this will perform    */
      /* the conversion of the next rank in sequencer.                          */
      /* Note: For this example, conversion is triggered by software start,     */
      /*       therefore "HAL_ADC_Start()" must be called for each conversion.  */
      /*       Since DMA transfer has been initiated previously by function     */
      /*       "HAL_ADC_Start_DMA()", this function will keep DMA transfer      */
      /*       active.                                                          */
      if (HAL_ADC_Start(&AdcHandle) != HAL_OK)
      {
        Error_Handler();
      }
      break;

    case MeasureSensors:
      if (ubSequenceCompleted == SET)
      {
        /* Computation of ADC conversions raw data to physical values */
        /* Note: ADC results are transferred into array "aADCxConvertedValues"  */
        /*       in the order of their rank in ADC sequencer.                   */
        raw_temp_value += aADCxConvertedValues[0];
        raw_volt_value += aADCxConvertedValues[1];

        /* Reset variable for next loop iteration */
        ubSequenceCompleted = RESET;
      }
      break;

    case StartupDelay: break;

  } // switch(adc_sensor_state)

  if (!adc_sensor_state && ++temp_count >= OVERSAMPLENR) { // 10 * 16 * 1/(16000000/64/256)  = 164ms.

    temp_count = 0;

    // Update the raw values if they've been read. Else we could be updating them during reading.
    if (!sens_meas_ready) set_current_sens_raw();

    raw_temp_value = 0;
    raw_volt_value = 0;

    #define TEMPDIR() ((PRINTHEAD_RAW_LO_TEMP) > (PRINTHEAD_RAW_HI_TEMP) ? -1 : 1)
    #define VOLTDIR() ((BATTERY_RAW_LO_VOLT) > (BATTERY_RAW_HI_VOLT) ? -1 : 1)

    int constexpr temp_dir = TEMPDIR();
    int constexpr volt_dir = VOLTDIR();

    const int16_t tdir = temp_dir, rawtemp = current_temperature_raw * tdir;
    if (rawtemp > maxttemp_raw * tdir) max_sens_error();
    if (rawtemp < minttemp_raw * tdir) {
          min_sens_error();
    }
    const int16_t vdir = volt_dir, rawvolt = current_voltage_raw * vdir;
    if (rawvolt > maxtvolt_raw * vdir) max_sens_error();
    if (rawvolt < mintvolt_raw * vdir) {
          min_sens_error();
    }
  } // temp_count >= OVERSAMPLENR

  // Go to the next state, up to SensorsReady
  adc_sensor_state = (ADCSensorState)((int(adc_sensor_state) + 1) % int(StartupDelay));

  extern volatile uint8_t e_hit;

  if (e_hit && endstops.enabled) {
    endstops.update();  // call endstop update routine
    e_hit--;
  }

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
//  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR));
  return ((raw * ((3.3 * 100.0) / 1024.0) / OVERSAMPLENR));
}

// For battery voltage measurement.
float Temperature::analog2volt(int raw) {
  if (battery_ttbl_map != NULL) {
    float voltage = 0;
    uint8_t i;
    short(*tt)[][2] = (short(*)[][2])(battery_ttbl_map);

    for (i = 1; i < battery_ttbllen_map; i++) {
      if ((short)((*tt)[i][0]) > raw) {
        voltage = (short)((*tt)[i - 1][1]) +
                  (raw - (short)((*tt)[i - 1][0])) *
                  (float)((short)((*tt)[i][1]) - (short)((*tt)[i - 1][1])) /
                  (float)((short)((*tt)[i][0]) - (short)((*tt)[i - 1][0]));
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == battery_ttbllen_map) voltage = (short)((*tt)[i - 1][1]);

    return voltage;
  }
//  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR));
  return ((raw * ((3.3 * 100.0) / 1024.0) / OVERSAMPLENR));
}

/**
 * Get raw temperatures
 */
void Temperature::set_current_sens_raw() {
  current_temperature_raw = raw_temp_value;
  current_voltage_raw = raw_volt_value;
  sens_meas_ready = true;
}

//
// Temperature Error Handlers
//
void Temperature::_sens_error(const char * const serial_msg, const char * const lcd_msg) {
  static bool killed = false;
  if (IsRunning()) {
//    SERIAL_ERROR_START();
//    serialprintPGM(serial_msg);
//    SERIAL_ERRORPGM(MSG_STOPPED_HEATER);
  }
  if (!killed) {
    Running = false;
    killed = true;
    kill(lcd_msg);
  }
  else
    disable_all_heaters(); // paranoia
}

void Temperature::max_sens_error() {
    _sens_error((const char *)(MSG_T_MAXTEMP), (const char *)(MSG_ERR_MAXTEMP));
}
void Temperature::min_sens_error() {
    _sens_error((const char *)(MSG_T_MINTEMP), (const char *)(MSG_ERR_MINTEMP));
}

void Temperature::disable_all_heaters() {
  // for sure our print job has stopped
  print_job_timer.stop();
}

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : ADC handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
  /* Report to main program that ADC sequencer has reached its end */
  ubSequenceCompleted = SET;
}

/**
  * @brief  Conversion DMA half-transfer callback in non blocking mode
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{

}

/**
  * @brief  ADC error callback in non blocking mode
  *        (ADC conversion with interruption or transfer by DMA)
  * @param  hadc: ADC handle
  * @retval None
  */
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
  /* In case of ADC error, call main error handler */
  Error_Handler();
}

