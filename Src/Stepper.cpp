/*
 * Stepper.cpp
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#include <Stepper.h>
#include "macros.h"

Stepper stepper; // Singleton

block_t* Stepper::current_block = NULL;  // A pointer to the block currently being traced

long Stepper::acceleration_time, Stepper::deceleration_time;

volatile long Stepper::count_position = 0;

uint8_t Stepper::last_direction_bits = 0;        // The next stepping-bits to be output
uint16_t Stepper::cleaning_buffer_counter = 0;

volatile signed char Stepper::count_direction = 1;

unsigned short Stepper::TIM2_ARR_nominal;

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  NVIC_EnableIRQ(TIM2_IRQn);
#define DISABLE_STEPPER_DRIVER_INTERRUPT() NVIC_DisableIRQ(TIM2_IRQn);

/**
 *         __________________________
 *        /|                        |\     _________________         ^
 *       / |                        | \   /|               |\        |
 *      /  |                        |  \ / |               | \       s
 *     /   |                        |   |  |               |  \      p
 *    /    |                        |   |  |               |   \     e
 *   +-----+------------------------+---+--+---------------+----+    e
 *   |               BLOCK 1            |      BLOCK 2          |    d
 *
 *                           time ----->
 *
 *  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
 *  first block->accelerate_until step_events_completed, then keeps going at constant speed until
 *  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
 *  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.
 */

void Stepper::wake_up() {
  // TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

/**
 * Block until all buffered steps are executed
 */
void Stepper::synchronize() { while (planner.blocks_queued()) idle(); }

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical steps.
 *
 * This allows get_position_mm to correctly
 * derive the current position later on.
 */
void Stepper::set_position(const long &a) {
  synchronize(); // Bad to set stepper counts in the middle of a move

  CRITICAL_SECTION_START;
  // default planning
  count_position = a;
  CRITICAL_SECTION_END;
}

void Stepper::init() {

  // Init Dir Pin
  #if HAS_MOTOR_DIR
    MOTOR_DIR_INIT;
  #endif

  // Init Enable Pin - stepper default to disabled.
  #if HAS_MOTOR_ENABLE
    MOTOR_ENABLE_INIT;
    if (!MOTOR_ENABLE_ON) MOTOR_ENABLE_WRITE(HIGH);
  #endif

  // Init endstops and pullups
  endstops.init();

  #define MOTOR_STEP_INIT SET_OUTPUT(MOTOR_STEP_PIN)
  #define MOTOR_STEP_WRITE(STATE) WRITE(MOTOR_STEP_PIN,STATE)
  #define MOTOR_STEP_READ READ(MOTOR_STEP_PIN)

  #define _WRITE_STEP(HIGHLOW) MOTOR_STEP_WRITE(HIGHLOW)
  #define _DISABLE() disable_MOTOR()

  #define MOTOR_INIT() \
    MOTOR_STEP_INIT(); \
    _WRITE_STEP(_INVERT_STEP_PIN()); \
    _DISABLE()

  // Init Step Pin
  #if HAS_MOTOR_STEP
    MOTOR_INIT();
  #endif

  TIM_HandleTypeDef htim2;

  TIM_Base_InitTypeDef tim2_Init;
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  tim2_Init.Prescaler = 8;
  tim2_Init.CounterMode = TIM_COUNTERMODE_UP;
  // Init Stepper ISR to 122 Hz for quick starting
  tim2_Init.Period = 0x4000;
  tim2_Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim2.Instance = TIM2;
  htim2.Init = tim2_Init;
  htim2.Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;

  if (HAL_ERROR == HAL_TIM_Base_Init(&htim2)) {
	  Error_Handler();
  }

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  //TIM2->PSC = 8; // 1/8 prescaler

  // Init Stepper ISR to 122 Hz for quick starting
  //TIM2->ARR = 0x4000;

  // Enable update interrupts
  //TIM2->DIER |= TIM_DIER_UIE;
  __HAL_TIM_ENABLE_IT(&htim2, TIM_IT_UPDATE);

  //TIM2->CNT = 0;
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  ENABLE_STEPPER_DRIVER_INTERRUPT();

  endstops.enable(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_directions(); // Init directions to last_direction_bits = 0
}

/**
 * Set the stepper direction
 */
void Stepper::set_directions() {

  #define SET_STEP_DIR \
    if (motor_direction()) { \
      APPLY_DIR(INVERT_DIR, false); \
      count_direction = -1; \
    } \
    else { \
      APPLY_DIR(!INVERT_DIR, false); \
      count_direction = 1; \
    }

  #if HAS_MOTOR_DIR
    SET_STEP_DIR(); // A
  #endif
}

/**
 * Stepper Driver Interrupt
 *
 * Directly pulses the stepper motors at high frequency.
 * Timer 1 runs at a base frequency of 2MHz, with this ISR using OCR1A compare mode.
 *
 * OCR1A   Frequency
 *     1     2 MHz
 *    50    40 KHz
 *   100    20 KHz - capped max rate
 *   200    10 KHz - nominal max rate
 *  2000     1 KHz - sleep rate
 *  4000   500  Hz - init rate
 */
void TIM2_IRQHandler(void)
{
    Stepper::isr();
}

#define _ENABLE_ISRs() do { \
	cli(); \
	if (thermalManager.in_temp_isr) \
		NVIC_DisableIRQ(TIM6_IRQn); \
	else \
	    NVIC_EnableIRQ(TIM6_IRQn); \
	ENABLE_STEPPER_DRIVER_INTERRUPT(); \
} while(0)

void Stepper::isr() {

  uint16_t ocr_val;

  #define ENDSTOP_NOMINAL_OCR_VAL 3000    // check endstops every 1.5ms to guarantee two stepper ISRs within 5ms for BLTouch
  #define OCR_VAL_TOLERANCE 1000          // First max delay is 2.0ms, last min delay is 0.5ms, all others 1.5ms

  // Disable Tim6 ISRs and enable global ISR again to capture UART events (incoming chars)
  NVIC_DisableIRQ(TIM6_IRQn); \
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  sei();

  #define _SPLIT(L) (ocr_val = (uint16_t)L)
  #define SPLIT(L) _SPLIT(L)

  if (cleaning_buffer_counter) {
    --cleaning_buffer_counter;
    current_block = NULL;
    planner.discard_current_block();
    __HAL_TIM_SET_AUTORELOAD(&htim2, 200); // Run at max speed - 10 KHz
    _ENABLE_ISRs(); // re-enable ISRs
    return;
  }

  // If there is no current block, attempt to pop one from the buffer
  if (!current_block) {
    // Anything in the buffer?
    current_block = planner.get_current_block();
    if (current_block) {
      trapezoid_generator_reset();

      // Initialize Bresenham counters to 1/2 the ceiling
      counter_X = counter_Y = counter_Z = counter_E = -(current_block->step_event_count >> 1);

      step_events_completed = 0;

      #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
        e_hit = 2; // Needed for the case an endstop is already triggered before the new move begins.
                   // No 'change' can be detected.
      #endif
    }
    else {
      _NEXT_ISR(2000); // Run at slow speed - 1 KHz
      _ENABLE_ISRs(); // re-enable ISRs
      return;
    }
  }

  // Update endstops state, if enabled
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    if (e_hit && ENDSTOPS_ENABLED) {
      endstops.update();
      e_hit--;
    }
  #else
    if (ENDSTOPS_ENABLED) endstops.update();
  #endif

  // Take multiple steps per interrupt (For high speed moves)
  bool all_steps_done = false;
  for (uint8_t i = step_loops; i--;) {
    #define _COUNTER(AXIS) counter_## AXIS
    #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
    #define _INVERT_STEP_PIN() INVERT_MOTOR_STEP_PIN

    // Advance the Bresenham counter; start a pulse if the axis needs a step
    #define PULSE_START(AXIS) \
      _COUNTER(AXIS) += current_block->steps[_AXIS(AXIS)]; \
      if (_COUNTER(AXIS) > 0) { _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0); }

    // Stop an active pulse, reset the Bresenham counter, update the position
    #define PULSE_STOP(AXIS) \
      if (_COUNTER(AXIS) > 0) { \
        _COUNTER(AXIS) -= current_block->step_event_count; \
        count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
        _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
      }

    /**
     * Estimate the number of cycles that the stepper logic already takes
     * up between the start and stop of the X stepper pulse.
     *
     * Currently this uses very modest estimates of around 5 cycles.
     * True values may be derived by careful testing.
     *
     * Once any delay is added, the cost of the delay code itself
     * may be subtracted from this value to get a more accurate delay.
     * Delays under 20 cycles (1.25�s) will be very accurate, using NOPs.
     * Longer delays use a loop. The resolution is 8 cycles.
     */
    #if HAS_X_STEP
      #define _CYCLE_APPROX_1 5
    #else
      #define _CYCLE_APPROX_1 0
    #endif
    #define _CYCLE_APPROX_2 _CYCLE_APPROX_1
    #if HAS_Y_STEP
      #define _CYCLE_APPROX_3 _CYCLE_APPROX_2 + 5
    #else
      #define _CYCLE_APPROX_3 _CYCLE_APPROX_2
    #endif
    #define _CYCLE_APPROX_4 _CYCLE_APPROX_3
    #if HAS_Z_STEP
      #define _CYCLE_APPROX_5 _CYCLE_APPROX_4 + 5
    #else
      #define _CYCLE_APPROX_5 _CYCLE_APPROX_4
    #endif
    #define _CYCLE_APPROX_6 _CYCLE_APPROX_5
    #define _CYCLE_APPROX_7 _CYCLE_APPROX_6 + 5

    #define CYCLES_EATEN_XYZE _CYCLE_APPROX_7
    #define EXTRA_CYCLES_XYZE (STEP_PULSE_CYCLES - (CYCLES_EATEN_XYZE))

    /**
     * If a minimum pulse time was specified get the timer 0 value.
     *
     * TCNT0 has an 8x prescaler, so it increments every 8 cycles.
     * That's every 0.5�s on 16MHz and every 0.4�s on 20MHz.
     * 20 counts of TCNT0 -by itself- is a good pulse delay.
     * 10�s = 160 or 200 cycles.
     */
    #if EXTRA_CYCLES_XYZE > 20
      uint32_t pulse_start = TCNT0;
    #endif

    #if HAS_X_STEP
      PULSE_START(X);
    #endif
    #if HAS_Y_STEP
      PULSE_START(Y);
    #endif
    #if HAS_Z_STEP
      PULSE_START(Z);
    #endif

    // For non-advance use linear interpolation for E also
    PULSE_START(E);

    // For minimum pulse time wait before stopping pulses
    #if EXTRA_CYCLES_XYZE > 20
      while (EXTRA_CYCLES_XYZE > (uint32_t)(TCNT0 - pulse_start) * (INT0_PRESCALER)) { /* nada */ }
      pulse_start = TCNT0;
    #elif EXTRA_CYCLES_XYZE > 0
      DELAY_NOPS(EXTRA_CYCLES_XYZE);
    #endif

    #if HAS_X_STEP
      PULSE_STOP(X);
    #endif
    #if HAS_Y_STEP
      PULSE_STOP(Y);
    #endif
    #if HAS_Z_STEP
      PULSE_STOP(Z);
    #endif

    PULSE_STOP(E);

    if (++step_events_completed >= current_block->step_event_count) {
      all_steps_done = true;
      break;
    }

    // For minimum pulse time wait after stopping pulses also
    #if EXTRA_CYCLES_XYZE > 20
      if (i) while (EXTRA_CYCLES_XYZE > (uint32_t)(TCNT0 - pulse_start) * (INT0_PRESCALER)) { /* nada */ }
    #elif EXTRA_CYCLES_XYZE > 0
      if (i) DELAY_NOPS(EXTRA_CYCLES_XYZE);
    #endif

  } // steps_loop

  // Calculate new timer value
  if (step_events_completed <= (uint32_t)current_block->accelerate_until) {

    MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
    acc_step_rate += current_block->initial_rate;

    // upper limit
    NOMORE(acc_step_rate, current_block->nominal_rate);

    // step_rate to timer interval
    const uint16_t timer = calc_timer(acc_step_rate);

    SPLIT(timer);  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);

    acceleration_time += timer;
  }
  else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
    uint16_t step_rate;
    MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);

    if (step_rate < acc_step_rate) { // Still decelerating?
      step_rate = acc_step_rate - step_rate;
      NOLESS(step_rate, current_block->final_rate);
    }
    else
      step_rate = current_block->final_rate;

    // step_rate to timer interval
    const uint16_t timer = calc_timer(step_rate);

    SPLIT(timer);  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);

    deceleration_time += timer;
  }
  else {

    SPLIT(TIM2_ARR_nominal);  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);

    // ensure we're running at the correct step rate, even if we just came off an acceleration
    step_loops = step_loops_nominal;
  }

  NOLESS(OCR1A, TCNT1 + 16);

  // If current block is finished, reset pointer
  if (all_steps_done) {
    current_block = NULL;
    planner.discard_current_block();
  }
  _ENABLE_ISRs(); // re-enable ISRs
}

