/*
 * Stepper.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef STEPPER_H_
#define STEPPER_H_

#include "macros.h"
#include "Conditionals.h"
#include <stm32l0xx_hal.h>

class Stepper {
public:
    static block_t* current_block;  // A pointer to the block currently being traced

    Stepper() {};

    //
    // Interrupt Service Routines
    //
    static void isr();

    //
    // The stepper subsystem goes to sleep when it runs out of things to execute. Call this
    // to notify the subsystem that it is time to go to work.
    //
    static void wake_up();

    //
    // Block until all buffered steps are executed
    //
    static void synchronize();

    //
    // Set the current position in steps
    //
    static void set_position(const long &a);

    //
    // The direction of a single motor
    //
    static FORCE_INLINE bool motor_direction() { return TEST(last_direction_bits, 0); }

    //
    // Set direction bits for all steppers
    //
    static void set_directions();

    //
    // Initialize stepper hardware
    //
    static void init();

private:
    static FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
      unsigned short timer;

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
        unsigned short table_address = (unsigned short)&speed_lookuptable_fast[(unsigned char)(step_rate >> 8)][0];
        unsigned char tmp_step_rate = (step_rate & 0x00FF);
        unsigned short gain = (unsigned short)(table_address + 2);
        MultiU16X8toH16(timer, tmp_step_rate, gain);
        timer = (unsigned short)(table_address) - timer;
      }
      else { // lower step rates
        unsigned short table_address = (unsigned short)&speed_lookuptable_slow[0][0];
        table_address += ((step_rate) >> 1) & 0xFFFC;
        timer = (unsigned short)(table_address);
        timer -= (((unsigned short)(table_address + 2) * (unsigned char)(step_rate & 0x0007)) >> 3);
      }
      if (timer < 100) { // (20kHz - this should never happen)
        timer = 100;
        MYSERIAL.print(MSG_STEPPER_TOO_HIGH);
        MYSERIAL.println(step_rate);
      }
      return timer;
    }

    //
    // Positions of stepper motors, in step units
    //
    static volatile long count_position[NUM_AXIS];

    static uint8_t last_direction_bits;        // The next stepping-bits to be output
    static uint16_t cleaning_buffer_counter;

    static long acceleration_time, deceleration_time;

    static unsigned short TIM2_ARR_nominal;

    //
    // Current direction of stepper motors (+1 or -1)
    //
    static volatile signed char count_direction[NUM_AXIS];

    // Initialize the trapezoid generator from the current block.
    // Called whenever a new block begins.
    static FORCE_INLINE void trapezoid_generator_reset() {
      static int8_t last_extruder = -1;

      if (current_block->direction_bits != last_direction_bits || current_block->active_extruder != last_extruder) {
        last_direction_bits = current_block->direction_bits;
        last_extruder = current_block->active_extruder;
        set_directions();
      }

      deceleration_time = 0;
      // step_rate to timer interval
      TIM2_ARR_nominal = calc_timer(current_block->nominal_rate);
      // make a note of the number of step loops required at nominal speed
      step_loops_nominal = step_loops;
      acc_step_rate = current_block->initial_rate;
      acceleration_time = calc_timer(acc_step_rate);
      __HAL_TIM_SET_AUTORELOAD(&htim2, acceleration_time);
    }

};

#endif /* STEPPER_H_ */
