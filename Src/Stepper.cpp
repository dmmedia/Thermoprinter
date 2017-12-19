/*
 * Stepper.cpp
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#include "gpio.h"
#include "main.h"
#include "macros.h"
#include "Stepper.h"
#include <stddef.h>
#include "SREGEmulation.h"
#include "Configuration.h"
#include "Temperature.h"
#include "Endstops.h"
#include "tim.h"

Stepper stepper { }; // Singleton

block_t* Stepper::current_block = NULL;  // A pointer to the block currently being traced

long Stepper::acceleration_time { }, Stepper::deceleration_time { };

volatile long Stepper::count_position { 0 };

uint8_t Stepper::last_direction_bits = 0;        // The next stepping-bits to be output
uint16_t Stepper::cleaning_buffer_counter = 0;

volatile signed char Stepper::count_direction = 1;

unsigned short Stepper::acc_step_rate { }; // needed for deceleration start point
uint8_t Stepper::step_loops { }, Stepper::step_loops_nominal { };
unsigned short Stepper::TIM6_ARR_nominal { };

long Stepper::counter_MOTOR { 0 };
volatile uint32_t Stepper::step_events_completed = 0; // The number of step events executed in the current block

volatile long Stepper::endstops_trigsteps { };

#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
  extern volatile uint8_t e_hit;
#endif

TIM_HandleTypeDef htim6 { };

#define MOTOR_APPLY_DIR(v,Q) MOTOR_DIR_WRITE(v)
#define MOTOR_APPLY_STEP(v,Q) MOTOR_STEP_WRITE(v)

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  NVIC_EnableIRQ(TIM6_DAC_IRQn);
#define DISABLE_STEPPER_DRIVER_INTERRUPT() NVIC_DisableIRQ(TIM6_DAC_IRQn);

// intRes = longIn1 * longIn2 >> 24
// uses:
// r26 to store 0
// r27 to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
// B0 A0 are bits 24-39 and are the returned value
// C1 B1 A1 is longIn1
// D2 C2 B2 A2 is longIn2
//
int MultiU24X32toH16(long longIn1, long longIn2) {
  return ((long long)longIn1 * longIn2) >> 24;
}
/*
#define MultiU24X32toH16(intRes, longIn1, longIn2) \
  asm volatile ( \
                 "clr r26 \n\t" \
                 "mul %A1, %B2 \n\t" \
                 "mov r27, r1 \n\t" \
                 "mul %B1, %C2 \n\t" \
                 "movw %A0, r0 \n\t" \
                 "mul %C1, %C2 \n\t" \
                 "add %B0, r0 \n\t" \
                 "mul %C1, %B2 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %A1, %C2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %B2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %C1, %A2 \n\t" \
                 "add r27, r0 \n\t" \
                 "adc %A0, r1 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %B1, %A2 \n\t" \
                 "add r27, r1 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "lsr r27 \n\t" \
                 "adc %A0, r26 \n\t" \
                 "adc %B0, r26 \n\t" \
                 "mul %D2, %A1 \n\t" \
                 "add %A0, r0 \n\t" \
                 "adc %B0, r1 \n\t" \
                 "mul %D2, %B1 \n\t" \
                 "add %B0, r0 \n\t" \
                 "clr r1 \n\t" \
                 : \
                 "=&r" (intRes) \
                 : \
                 "d" (longIn1), \
                 "d" (longIn2) \
                 : \
                 "r26" , "r27" \
               )
*/

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

/**
 * Set the stepper direction
 */
void Stepper::set_directions() {

  if (motor_direction()) {
    MOTOR_APPLY_DIR(INVERT_MOTOR_DIR, false);
    count_direction = -1;
  } else {
    MOTOR_APPLY_DIR(INVERT_MOTOR_DIR == GPIO_PIN_RESET ? GPIO_PIN_SET : GPIO_PIN_RESET, false);
    count_direction = 1;
  }
}

/**
 * Stepper Driver Interrupt
 *
 * Directly pulses the stepper motors at high frequency.
 * Timer TIM6 runs at a base frequency of 2MHz, with this ISR using ARR compare mode.
 *
 * ARR   Frequency
 *     1     2 MHz
 *    50    40 KHz
 *   100    20 KHz - capped max rate
 *   200    10 KHz - nominal max rate
 *  2000     1 KHz - sleep rate
 *  4000   500  Hz - init rate
 */
void TIM6_IRQHandler(void)
{
    Stepper::isr();
}

#define _ENABLE_ISRs() do { \
	cli(); \
	if (thermalManager.in_temp_isr) \
		NVIC_DisableIRQ(TIM2_IRQn); \
	else \
	    NVIC_EnableIRQ(TIM2_IRQn); \
	ENABLE_STEPPER_DRIVER_INTERRUPT(); \
} while(0)

void Stepper::isr() {

  uint16_t ocr_val { };

  #define ENDSTOP_NOMINAL_OCR_VAL 3000    // check endstops every 1.5ms to guarantee two stepper ISRs within 5ms for BLTouch
  #define OCR_VAL_TOLERANCE 1000          // First max delay is 2.0ms, last min delay is 0.5ms, all others 1.5ms

  // Disable Tim6 ISRs and enable global ISR again to capture UART events (incoming chars)
  NVIC_DisableIRQ(TIM2_IRQn); \
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  sei();

  #define _SPLIT(L) (ocr_val = (uint16_t)L)
  #define SPLIT(L) _SPLIT(L)

  if (cleaning_buffer_counter) {
    --cleaning_buffer_counter;
    current_block = NULL;
    planner.discard_current_block();
    TIM_SET_AUTORELOAD(&htim6, 200); // Run at max speed - 10 KHz
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
      counter_MOTOR = -(current_block->step_event_count >> 1);

      step_events_completed = 0;

      #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
        e_hit = 2; // Needed for the case an endstop is already triggered before the new move begins.
                   // No 'change' can be detected.
      #endif
    }
    else {
      TIM_SET_AUTORELOAD(&htim6, 2000); // Run at slow speed - 1 KHz
      _ENABLE_ISRs(); // re-enable ISRs
      return;
    }
  }

  // Update endstops state, if enabled
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    if (e_hit && endstops.enabled) {
      endstops.update();
      e_hit--;
    }
  #else
    if (endstops.enabled) endstops.update();
  #endif

  // Take multiple steps per interrupt (For high speed moves)
  bool all_steps_done { false };
  for (uint8_t i = step_loops; i--;) {
    #define _COUNTER() counter_MOTOR
    #define _APPLY_STEP() MOTOR_APPLY_STEP
    #define _INVERT_STEP_PIN() INVERT_MOTOR_STEP_PIN

    // Advance the Bresenham counter; start a pulse if the axis needs a step
    #define PULSE_START() \
      _COUNTER() += current_block->steps; \
      if (_COUNTER() > 0) { _APPLY_STEP()(_INVERT_STEP_PIN() == GPIO_PIN_SET ? GPIO_PIN_RESET : GPIO_PIN_SET,0); }

    // Stop an active pulse, reset the Bresenham counter, update the position
    #define PULSE_STOP() \
      if (_COUNTER() > 0) { \
        _COUNTER() -= current_block->step_event_count; \
        count_position += count_direction; \
        _APPLY_STEP()(_INVERT_STEP_PIN(),0); \
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
     * Delays under 20 cycles (1.25탎) will be very accurate, using NOPs.
     * Longer delays use a loop. The resolution is 8 cycles.
     */
    #define EXTRA_CYCLES_XYZE (STEP_PULSE_CYCLES - 10)

    /**
     * If a minimum pulse time was specified get the timer TIM2 value.
     *
     * TIM2->CNT has an 8x prescaler, so it increments every 8 cycles.
     * That's every 0.5탎 on 16MHz and every 0.4탎 on 20MHz.
     * 20 counts of TCNT0 -by itself- is a good pulse delay.
     * 10탎 = 160 or 200 cycles.
     */
    #if EXTRA_CYCLES_XYZE > 20
      uint32_t pulse_start = __HAL_TIM_GET_COUNTER(&htim2);
    #endif

    PULSE_START();

    // For minimum pulse time wait before stopping pulses
    #if EXTRA_CYCLES_XYZE > 20
      while (EXTRA_CYCLES_XYZE > (uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - pulse_start) * (8)) { }
      pulse_start = __HAL_TIM_GET_COUNTER(&htim2);
    #elif EXTRA_CYCLES_XYZE > 0
      DELAY_NOPS(EXTRA_CYCLES_XYZE);
    #endif

    PULSE_STOP();

    if (++step_events_completed >= current_block->step_event_count) {
      all_steps_done = true;
      break;
    }

    // For minimum pulse time wait after stopping pulses also
    #if EXTRA_CYCLES_XYZE > 20
      if (i) while (EXTRA_CYCLES_XYZE > (uint32_t)(__HAL_TIM_GET_COUNTER(&htim2) - pulse_start) * (8)) { }
    #elif EXTRA_CYCLES_XYZE > 0
      if (i) DELAY_NOPS(EXTRA_CYCLES_XYZE);
    #endif

  } // steps_loop

  // Calculate new timer value
  if (step_events_completed <= (uint32_t)current_block->accelerate_until) {

	acc_step_rate = MultiU24X32toH16(acceleration_time, current_block->acceleration_rate);
    acc_step_rate += current_block->initial_rate;

    // upper limit
    NOMORE(acc_step_rate, current_block->nominal_rate);

    // step_rate to timer interval
    const uint16_t timer = calc_timer(acc_step_rate);

    SPLIT(timer);  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
    TIM_SET_AUTORELOAD(&htim6, ocr_val);

    acceleration_time += timer;
  }
  else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
    uint16_t step_rate { };
    step_rate = MultiU24X32toH16(deceleration_time, current_block->acceleration_rate);

    if (step_rate < acc_step_rate) { // Still decelerating?
      step_rate = acc_step_rate - step_rate;
      NOLESS(step_rate, current_block->final_rate);
    }
    else
      step_rate = current_block->final_rate;

    // step_rate to timer interval
    const uint16_t timer = calc_timer(step_rate);

    SPLIT(timer);  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
    TIM_SET_AUTORELOAD(&htim6, ocr_val);

    deceleration_time += timer;
  }
  else {

    SPLIT(TIM6_ARR_nominal);  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
    TIM_SET_AUTORELOAD(&htim6, ocr_val);

    // ensure we're running at the correct step rate, even if we just came off an acceleration
    step_loops = step_loops_nominal;
  }

  NOLESS(TIM_GET_AUTORELOAD(&htim6), TIM_GET_COUNTER(&htim6) + 16);

  // If current block is finished, reset pointer
  if (all_steps_done) {
    current_block = NULL;
    planner.discard_current_block();
  }
  _ENABLE_ISRs(); // re-enable ISRs
}

void Stepper::endstop_triggered() {

  endstops_trigsteps = count_position;

  kill_current_block();
}

void Stepper::init() {

  // Init Dir Pin
  SET_OUTPUT(MOTOR_DIR);

  // Init Enable Pin - stepper default to disabled.
  SET_OUTPUT(MOTOR_ENABLE);
  if (!MOTOR_ENABLE_ON) GPIO_WRITE_PIN(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN, GPIO_PIN_SET);

  // Init endstops and pullups
  endstops.init();

  // Init Step Pin
  SET_OUTPUT(MOTOR_STEP);
  WRITE(MOTOR_STEP, GPIO_PIN_RESET);
  disable_MOTOR();

  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer
  // frequency on a 16MHz MCU. If you are going to change this, be
  // sure to regenerate speed_lookuptable.h with
  // create_speed_lookuptable.py
  TIM_Base_InitTypeDef tim6_Init { };
  tim6_Init.Prescaler = 8;
  tim6_Init.CounterMode = TIM_COUNTERMODE_UP;
  // Init Stepper ISR to 122 Hz for quick starting
  tim6_Init.Period = 0x4000;
  tim6_Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

  htim6.Instance = TIM6;
  htim6.Init = tim6_Init;
  htim6.Channel = TIM_ACTIVE_CHANNEL_CLEARED;

  if (STATUS_ERROR == TIM_Base_Init(&htim6)) {
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
  //TIM6->DIER |= TIM_DIER_UIE;
  TIM_ENABLE_IT(&htim6, TIM_DIER_UIE);

  // zero counter for start
  //TIM6->CNT = 0;
  TIM_RESET_COUNTER(&htim6);

  ENABLE_STEPPER_DRIVER_INTERRUPT();

  endstops.enable(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_directions(); // Init directions to last_direction_bits = 0
}

FORCE_INLINE void Stepper::trapezoid_generator_reset() {
  if (current_block->direction_bits != last_direction_bits) {
    last_direction_bits = current_block->direction_bits;
    set_directions();
  }

  deceleration_time = 0;
  // step_rate to timer interval
  TIM6_ARR_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  TIM_SET_AUTORELOAD(&htim6, acceleration_time);
}

unsigned short MultiU16X8toH16(unsigned char charIn1, unsigned int intIn2) {
	return (unsigned short)((intIn2 * charIn1) >> 16);
}

FORCE_INLINE unsigned short Stepper::calc_timer(unsigned short step_rate) {
  unsigned short timer { };

  NOMORE(step_rate, MAX_STEP_FREQUENCY);

  if (step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate >>= 2;
    step_loops = 4;
  }
  else if (step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate >>= 1;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }

  NOLESS(step_rate, SystemCoreClock / 500000);
  step_rate -= SystemCoreClock / 500000; // Correct for minimal speed
  if (step_rate >= (8 * 256)) { // higher step rate
    uint16_t *table_address = (uint16_t *)&speed_lookuptable_fast[(uint8_t) (step_rate >> 8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00FF);
    uint16_t gain = *(table_address + 1);
    timer = MultiU16X8toH16(tmp_step_rate, gain);
    timer = (*table_address) - timer;
  }
  else { // lower step rates
    uint16_t *table_address = (uint16_t *)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate) >> 1) & 0xFFFFFFFC;
    timer = *table_address;
    timer -= (((*(table_address + 1)) * (unsigned char)(step_rate & 0x0007)) >> 3);
  }
  if (timer < 100) { // (20kHz - this should never happen)
    timer = 100;
  }
  return timer;
}

FORCE_INLINE bool Stepper::motor_direction() { return TEST(last_direction_bits, 0); }

inline void Stepper::kill_current_block() {
  step_events_completed = current_block->step_event_count;
}
