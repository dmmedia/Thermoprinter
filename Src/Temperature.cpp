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
int16_t Temperature::minttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP, HEATER_3_RAW_LO_TEMP, HEATER_4_RAW_LO_TEMP),
        Temperature::maxttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP, HEATER_3_RAW_HI_TEMP, HEATER_4_RAW_HI_TEMP),
        Temperature::minttemp[HOTENDS] = { 0 },
        Temperature::maxttemp[HOTENDS] = ARRAY_BY_HOTENDS1(16383);

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_heater()
 */
void Temperature::init() {

  // Finish init of mult hotend arrays
  HOTEND_LOOP() maxttemp[e] = maxttemp[0];

  ADC_Config();

  // Set analog inputs
  DIDR0 = 0;
  #ifdef DIDR2
    DIDR2 = 0;
  #endif
  #if HAS_TEMP_0
    ANALOG_SELECT(TEMP_0_PIN);
  #endif
  #if HAS_TEMP_1
    ANALOG_SELECT(TEMP_1_PIN);
  #endif
  #if HAS_TEMP_2
    ANALOG_SELECT(TEMP_2_PIN);
  #endif
  #if HAS_TEMP_3
    ANALOG_SELECT(TEMP_3_PIN);
  #endif
  #if HAS_TEMP_4
    ANALOG_SELECT(TEMP_4_PIN);
  #endif
  #if HAS_TEMP_BED
    ANALOG_SELECT(TEMP_BED_PIN);
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    ANALOG_SELECT(FILWIDTH_PIN);
  #endif

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  SBI(TIMSK0, OCIE0B);

  // Wait for temperature measurement to settle
  HAL_Delay(250);

  #define TEMP_MIN_ROUTINE(NR) \
    minttemp[NR] = HEATER_ ##NR## _MINTEMP; \
    while (analog2temp(minttemp_raw[NR], NR) < HEATER_ ##NR## _MINTEMP) { \
      if (HEATER_ ##NR## _RAW_LO_TEMP < HEATER_ ##NR## _RAW_HI_TEMP) \
        minttemp_raw[NR] += OVERSAMPLENR; \
      else \
        minttemp_raw[NR] -= OVERSAMPLENR; \
    }
  #define TEMP_MAX_ROUTINE(NR) \
    maxttemp[NR] = HEATER_ ##NR## _MAXTEMP; \
    while (analog2temp(maxttemp_raw[NR], NR) > HEATER_ ##NR## _MAXTEMP) { \
      if (HEATER_ ##NR## _RAW_LO_TEMP < HEATER_ ##NR## _RAW_HI_TEMP) \
        maxttemp_raw[NR] -= OVERSAMPLENR; \
      else \
        maxttemp_raw[NR] += OVERSAMPLENR; \
    }

  #ifdef HEATER_0_MINTEMP
    TEMP_MIN_ROUTINE(0);
  #endif
  #ifdef HEATER_0_MAXTEMP
    TEMP_MAX_ROUTINE(0);
  #endif
  #ifdef BED_MINTEMP
    while (analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_minttemp_raw += OVERSAMPLENR;
      #else
        bed_minttemp_raw -= OVERSAMPLENR;
      #endif
    }
  #endif // BED_MINTEMP
  #ifdef BED_MAXTEMP
    while (analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_maxttemp_raw -= OVERSAMPLENR;
      #else
        bed_maxttemp_raw += OVERSAMPLENR;
      #endif
    }
  #endif // BED_MAXTEMP

  #if ENABLED(PROBING_HEATERS_OFF)
    paused = false;
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
  AdcHandle.Instance = ADCx;

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
  sConfig.Channel      = ADCx_CHANNEL_TEMPERATURE;

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
  sConfig.Channel      = ADCx_CHANNEL_VOLTAGE;

  if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

}

/**
 * Timer 0 is shared with millies so don't change the prescaler.
 *
 * This ISR uses the compare method so it runs at the base
 * frequency (16 MHz / 64 / 256 = 976.5625 Hz), but at the TCNT0 set
 * in OCR0B above (128 or halfway between OVFs).
 *
 *  - Manage PWM to all the heaters and fan
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - Check new temperature values for MIN/MAX errors (kill on error)
 *  - Step the babysteps value for each axis towards 0
 *  - For PINS_DEBUGGING, monitor and report endstop pins
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 */
ISR(TIMER0_COMPB_vect) { Temperature::isr(); }

volatile bool Temperature::in_temp_isr = false;

void Temperature::isr() {
  // The stepper ISR can interrupt this ISR. When it does it re-enables this ISR
  // at the end of its run, potentially causing re-entry. This flag prevents it.
  if (in_temp_isr) return;
  in_temp_isr = true;

  // Allow UART and stepper ISRs
  CBI(TIMSK0, OCIE0B); //Disable Temperature ISR
  sei();

  static int8_t temp_count = -1;
  static ADCSensorState adc_sensor_state = StartupDelay;
  static uint8_t pwm_count = _BV(SOFT_PWM_SCALE);
  // avoid multiple loads of pwm_count
  uint8_t pwm_count_tmp = pwm_count;
  #if ENABLED(ADC_KEYPAD)
    static unsigned int raw_ADCKey_value = 0;
  #endif

  // Static members for each heater
  #if ENABLED(SLOW_PWM_HEATERS)
    static uint8_t slow_pwm_count = 0;
    #define ISR_STATICS(n) \
      static uint8_t soft_pwm_count_ ## n, \
                     state_heater_ ## n = 0, \
                     state_timer_heater_ ## n = 0
  #else
    #define ISR_STATICS(n) static uint8_t soft_pwm_count_ ## n = 0
  #endif

  // Statics per heater
  ISR_STATICS(0);
  #if HOTENDS > 1
    ISR_STATICS(1);
    #if HOTENDS > 2
      ISR_STATICS(2);
      #if HOTENDS > 3
        ISR_STATICS(3);
        #if HOTENDS > 4
          ISR_STATICS(4);
        #endif // HOTENDS > 4
      #endif // HOTENDS > 3
    #endif // HOTENDS > 2
  #endif // HOTENDS > 1
  #if HAS_HEATER_BED
    ISR_STATICS(BED);
  #endif

  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    static unsigned long raw_filwidth_value = 0;
  #endif

  #if DISABLED(SLOW_PWM_HEATERS)
    constexpr uint8_t pwm_mask =
      #if ENABLED(SOFT_PWM_DITHER)
        _BV(SOFT_PWM_SCALE) - 1
      #else
        0
      #endif
    ;

    /**
     * Standard PWM modulation
     */
    if (pwm_count_tmp >= 127) {
      pwm_count_tmp -= 127;
      soft_pwm_count_0 = (soft_pwm_count_0 & pwm_mask) + soft_pwm_amount[0];
      WRITE_HEATER_0(soft_pwm_count_0 > pwm_mask ? HIGH : LOW);
      #if HOTENDS > 1
        soft_pwm_count_1 = (soft_pwm_count_1 & pwm_mask) + soft_pwm_amount[1];
        WRITE_HEATER_1(soft_pwm_count_1 > pwm_mask ? HIGH : LOW);
        #if HOTENDS > 2
          soft_pwm_count_2 = (soft_pwm_count_2 & pwm_mask) + soft_pwm_amount[2];
          WRITE_HEATER_2(soft_pwm_count_2 > pwm_mask ? HIGH : LOW);
          #if HOTENDS > 3
            soft_pwm_count_3 = (soft_pwm_count_3 & pwm_mask) + soft_pwm_amount[3];
            WRITE_HEATER_3(soft_pwm_count_3 > pwm_mask ? HIGH : LOW);
            #if HOTENDS > 4
              soft_pwm_count_4 = (soft_pwm_count_4 & pwm_mask) + soft_pwm_amount[4];
              WRITE_HEATER_4(soft_pwm_count_4 > pwm_mask ? HIGH : LOW);
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
      #endif // HOTENDS > 1

      #if HAS_HEATER_BED
        soft_pwm_count_BED = (soft_pwm_count_BED & pwm_mask) + soft_pwm_amount_bed;
        WRITE_HEATER_BED(soft_pwm_count_BED > pwm_mask ? HIGH : LOW);
      #endif

      #if ENABLED(FAN_SOFT_PWM)
        #if HAS_FAN0
          soft_pwm_count_fan[0] = (soft_pwm_count_fan[0] & pwm_mask) + soft_pwm_amount_fan[0] >> 1;
          WRITE_FAN(soft_pwm_count_fan[0] > pwm_mask ? HIGH : LOW);
        #endif
        #if HAS_FAN1
          soft_pwm_count_fan[1] = (soft_pwm_count_fan[1] & pwm_mask) + soft_pwm_amount_fan[1] >> 1;
          WRITE_FAN1(soft_pwm_count_fan[1] > pwm_mask ? HIGH : LOW);
        #endif
        #if HAS_FAN2
          soft_pwm_count_fan[2] = (soft_pwm_count_fan[2] & pwm_mask) + soft_pwm_amount_fan[2] >> 1;
          WRITE_FAN2(soft_pwm_count_fan[2] > pwm_mask ? HIGH : LOW);
        #endif
      #endif
    }
    else {
      if (soft_pwm_count_0 <= pwm_count_tmp) WRITE_HEATER_0(LOW);
      #if HOTENDS > 1
        if (soft_pwm_count_1 <= pwm_count_tmp) WRITE_HEATER_1(LOW);
        #if HOTENDS > 2
          if (soft_pwm_count_2 <= pwm_count_tmp) WRITE_HEATER_2(LOW);
          #if HOTENDS > 3
            if (soft_pwm_count_3 <= pwm_count_tmp) WRITE_HEATER_3(LOW);
            #if HOTENDS > 4
              if (soft_pwm_count_4 <= pwm_count_tmp) WRITE_HEATER_4(LOW);
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
      #endif // HOTENDS > 1

      #if HAS_HEATER_BED
        if (soft_pwm_count_BED <= pwm_count_tmp) WRITE_HEATER_BED(LOW);
      #endif

      #if ENABLED(FAN_SOFT_PWM)
        #if HAS_FAN0
          if (soft_pwm_count_fan[0] <= pwm_count_tmp) WRITE_FAN(LOW);
        #endif
        #if HAS_FAN1
          if (soft_pwm_count_fan[1] <= pwm_count_tmp) WRITE_FAN1(LOW);
        #endif
        #if HAS_FAN2
          if (soft_pwm_count_fan[2] <= pwm_count_tmp) WRITE_FAN2(LOW);
        #endif
      #endif
    }

    // SOFT_PWM_SCALE to frequency:
    //
    // 0: 16000000/64/256/128 =   7.6294 Hz
    // 1:                / 64 =  15.2588 Hz
    // 2:                / 32 =  30.5176 Hz
    // 3:                / 16 =  61.0352 Hz
    // 4:                /  8 = 122.0703 Hz
    // 5:                /  4 = 244.1406 Hz
    pwm_count = pwm_count_tmp + _BV(SOFT_PWM_SCALE);

  #else // SLOW_PWM_HEATERS

    /**
     * SLOW PWM HEATERS
     *
     * For relay-driven heaters
     */
    #ifndef MIN_STATE_TIME
      #define MIN_STATE_TIME 16 // MIN_STATE_TIME * 65.5 = time in milliseconds
    #endif

    // Macros for Slow PWM timer logic
    #define _SLOW_PWM_ROUTINE(NR, src) \
      soft_pwm_ ##NR = src; \
      if (soft_pwm_ ##NR > 0) { \
        if (state_timer_heater_ ##NR == 0) { \
          if (state_heater_ ##NR == 0) state_timer_heater_ ##NR = MIN_STATE_TIME; \
          state_heater_ ##NR = 1; \
          WRITE_HEATER_ ##NR(1); \
        } \
      } \
      else { \
        if (state_timer_heater_ ##NR == 0) { \
          if (state_heater_ ##NR == 1) state_timer_heater_ ##NR = MIN_STATE_TIME; \
          state_heater_ ##NR = 0; \
          WRITE_HEATER_ ##NR(0); \
        } \
      }
    #define SLOW_PWM_ROUTINE(n) _SLOW_PWM_ROUTINE(n, soft_pwm_amount[n])

    #define PWM_OFF_ROUTINE(NR) \
      if (soft_pwm_ ##NR < slow_pwm_count) { \
        if (state_timer_heater_ ##NR == 0) { \
          if (state_heater_ ##NR == 1) state_timer_heater_ ##NR = MIN_STATE_TIME; \
          state_heater_ ##NR = 0; \
          WRITE_HEATER_ ##NR (0); \
        } \
      }

    if (slow_pwm_count == 0) {

      SLOW_PWM_ROUTINE(0);
      #if HOTENDS > 1
        SLOW_PWM_ROUTINE(1);
        #if HOTENDS > 2
          SLOW_PWM_ROUTINE(2);
          #if HOTENDS > 3
            SLOW_PWM_ROUTINE(3);
            #if HOTENDS > 4
              SLOW_PWM_ROUTINE(4);
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
      #endif // HOTENDS > 1
      #if HAS_HEATER_BED
        _SLOW_PWM_ROUTINE(BED, soft_pwm_amount_bed); // BED
      #endif

    } // slow_pwm_count == 0

    PWM_OFF_ROUTINE(0);
    #if HOTENDS > 1
      PWM_OFF_ROUTINE(1);
      #if HOTENDS > 2
        PWM_OFF_ROUTINE(2);
        #if HOTENDS > 3
          PWM_OFF_ROUTINE(3);
          #if HOTENDS > 4
            PWM_OFF_ROUTINE(4);
          #endif // HOTENDS > 4
        #endif // HOTENDS > 3
      #endif // HOTENDS > 2
    #endif // HOTENDS > 1
    #if HAS_HEATER_BED
      PWM_OFF_ROUTINE(BED); // BED
    #endif

    #if ENABLED(FAN_SOFT_PWM)
      if (pwm_count_tmp >= 127) {
        pwm_count_tmp = 0;
        #if HAS_FAN0
          soft_pwm_count_fan[0] = soft_pwm_amount_fan[0] >> 1;
          WRITE_FAN(soft_pwm_count_fan[0] > 0 ? HIGH : LOW);
        #endif
        #if HAS_FAN1
          soft_pwm_count_fan[1] = soft_pwm_amount_fan[1] >> 1;
          WRITE_FAN1(soft_pwm_count_fan[1] > 0 ? HIGH : LOW);
        #endif
        #if HAS_FAN2
          soft_pwm_count_fan[2] = soft_pwm_amount_fan[2] >> 1;
          WRITE_FAN2(soft_pwm_count_fan[2] > 0 ? HIGH : LOW);
        #endif
      }
      #if HAS_FAN0
        if (soft_pwm_count_fan[0] <= pwm_count_tmp) WRITE_FAN(LOW);
      #endif
      #if HAS_FAN1
        if (soft_pwm_count_fan[1] <= pwm_count_tmp) WRITE_FAN1(LOW);
      #endif
      #if HAS_FAN2
        if (soft_pwm_count_fan[2] <= pwm_count_tmp) WRITE_FAN2(LOW);
      #endif
    #endif // FAN_SOFT_PWM

    // SOFT_PWM_SCALE to frequency:
    //
    // 0: 16000000/64/256/128 =   7.6294 Hz
    // 1:                / 64 =  15.2588 Hz
    // 2:                / 32 =  30.5176 Hz
    // 3:                / 16 =  61.0352 Hz
    // 4:                /  8 = 122.0703 Hz
    // 5:                /  4 = 244.1406 Hz
    pwm_count = pwm_count_tmp + _BV(SOFT_PWM_SCALE);

    // increment slow_pwm_count only every 64th pwm_count,
    // i.e. yielding a PWM frequency of 16/128 Hz (8s).
    if (((pwm_count >> SOFT_PWM_SCALE) & 0x3F) == 0) {
      slow_pwm_count++;
      slow_pwm_count &= 0x7F;

      if (state_timer_heater_0 > 0) state_timer_heater_0--;
      #if HOTENDS > 1
        if (state_timer_heater_1 > 0) state_timer_heater_1--;
        #if HOTENDS > 2
          if (state_timer_heater_2 > 0) state_timer_heater_2--;
          #if HOTENDS > 3
            if (state_timer_heater_3 > 0) state_timer_heater_3--;
            #if HOTENDS > 4
              if (state_timer_heater_4 > 0) state_timer_heater_4--;
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
      #endif // HOTENDS > 1
      #if HAS_HEATER_BED
        if (state_timer_heater_BED > 0) state_timer_heater_BED--;
      #endif
    } // ((pwm_count >> SOFT_PWM_SCALE) & 0x3F) == 0

  #endif // SLOW_PWM_HEATERS

  //
  // Update lcd buttons 488 times per second
  //
  static bool do_buttons;
  if ((do_buttons ^= true)) lcd_buttons_update();

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

    #if HAS_TEMP_BED
      case PrepareTemp_BED:
        START_ADC(TEMP_BED_PIN);
        break;
      case MeasureTemp_BED:
        raw_temp_bed_value += ADC;
        break;
    #endif

    #if HAS_TEMP_1
      case PrepareTemp_1:
        START_ADC(TEMP_1_PIN);
        break;
      case MeasureTemp_1:
        raw_temp_value[1] += ADC;
        break;
    #endif

    #if HAS_TEMP_2
      case PrepareTemp_2:
        START_ADC(TEMP_2_PIN);
        break;
      case MeasureTemp_2:
        raw_temp_value[2] += ADC;
        break;
    #endif

    #if HAS_TEMP_3
      case PrepareTemp_3:
        START_ADC(TEMP_3_PIN);
        break;
      case MeasureTemp_3:
        raw_temp_value[3] += ADC;
        break;
    #endif

    #if HAS_TEMP_4
      case PrepareTemp_4:
        START_ADC(TEMP_4_PIN);
        break;
      case MeasureTemp_4:
        raw_temp_value[4] += ADC;
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

    #if ENABLED(ADC_KEYPAD)
      case Prepare_ADC_KEY:
        START_ADC(ADC_KEYPAD_PIN);
        break;
      case Measure_ADC_KEY:
        if (ADCKey_count < 16) {
          raw_ADCKey_value = ADC;
          if (raw_ADCKey_value > 900) {
            //ADC Key release
            ADCKey_count = 0;
            current_ADCKey_raw = 0;
          }
          else {
            current_ADCKey_raw += raw_ADCKey_value;
            ADCKey_count++;
          }
        }
        break;
    #endif // ADC_KEYPAD

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

    #define TEMPDIR(N) ((HEATER_##N##_RAW_LO_TEMP) > (HEATER_##N##_RAW_HI_TEMP) ? -1 : 1)

    int constexpr temp_dir[] = {
      #if ENABLED(HEATER_0_USES_MAX6675)
         0
      #else
        TEMPDIR(0)
      #endif
      #if HOTENDS > 1
        , TEMPDIR(1)
        #if HOTENDS > 2
          , TEMPDIR(2)
          #if HOTENDS > 3
            , TEMPDIR(3)
            #if HOTENDS > 4
              , TEMPDIR(4)
            #endif // HOTENDS > 4
          #endif // HOTENDS > 3
        #endif // HOTENDS > 2
      #endif // HOTENDS > 1
    };

    for (uint8_t e = 0; e < COUNT(temp_dir); e++) {
      const int16_t tdir = temp_dir[e], rawtemp = current_temperature_raw[e] * tdir;
      if (rawtemp > maxttemp_raw[e] * tdir && target_temperature[e] > 0) max_temp_error(e);
      if (rawtemp < minttemp_raw[e] * tdir && !is_preheating(e) && target_temperature[e] > 0) {
        #ifdef MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED
          if (++consecutive_low_temperature_error[e] >= MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED)
        #endif
            min_temp_error(e);
      }
      #ifdef MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED
        else
          consecutive_low_temperature_error[e] = 0;
      #endif
    }

    #if HAS_TEMP_BED
      #if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
        #define GEBED <=
      #else
        #define GEBED >=
      #endif
      if (current_temperature_bed_raw GEBED bed_maxttemp_raw && target_temperature_bed > 0) max_temp_error(-1);
      if (bed_minttemp_raw GEBED current_temperature_bed_raw && target_temperature_bed > 0) min_temp_error(-1);
    #endif

  } // temp_count >= OVERSAMPLENR

  // Go to the next state, up to SensorsReady
  adc_sensor_state = (ADCSensorState)((int(adc_sensor_state) + 1) % int(StartupDelay));

  #if ENABLED(BABYSTEPPING)
    LOOP_XYZ(axis) {
      const int curTodo = babystepsTodo[axis]; // get rid of volatile for performance
      if (curTodo) {
        stepper.babystep((AxisEnum)axis, curTodo > 0);
        if (curTodo > 0) babystepsTodo[axis]--;
                    else babystepsTodo[axis]++;
      }
    }
  #endif // BABYSTEPPING

  #if ENABLED(PINS_DEBUGGING)
    extern bool endstop_monitor_flag;
    // run the endstop monitor at 15Hz
    static uint8_t endstop_monitor_count = 16;  // offset this check from the others
    if (endstop_monitor_flag) {
      endstop_monitor_count += _BV(1);  //  15 Hz
      endstop_monitor_count &= 0x7F;
      if (!endstop_monitor_count) endstop_monitor();  // report changes in endstop status
    }
  #endif

  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)

    extern volatile uint8_t e_hit;

    if (e_hit && ENDSTOPS_ENABLED) {
      endstops.update();  // call endstop update routine
      e_hit--;
    }
  #endif

  cli();
  in_temp_isr = false;
  SBI(TIMSK0, OCIE0B); //re-enable Temperature ISR
}
