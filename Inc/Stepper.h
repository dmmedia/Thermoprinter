/*
 * Stepper.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef STEPPER_H_
#define STEPPER_H_

#include "macros.h"

#define E_STEP_WRITE(v) E0_STEP_WRITE(v)
#define NORM_E_DIR() E0_DIR_WRITE(!INVERT_E0_DIR)
#define REV_E_DIR() E0_DIR_WRITE(INVERT_E0_DIR)

class Stepper {
public:
	Stepper() {};

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
    static void set_position(const long &a, const long &b, const long &c, const long &e);
    static void set_position(const AxisEnum &a, const long &v);

    //
    // The direction of a single motor
    //
    static FORCE_INLINE bool motor_direction(AxisEnum axis) { return TEST(last_direction_bits, axis); }

    //
    // Set direction bits for all steppers
    //
    static void set_directions();

    //
    // Initialize stepper hardware
    //
    static void init();

private:
    //
    // Positions of stepper motors, in step units
    //
    static volatile long count_position[NUM_AXIS];

    static uint8_t last_direction_bits;        // The next stepping-bits to be output

    //
    // Current direction of stepper motors (+1 or -1)
    //
    static volatile signed char count_direction[NUM_AXIS];

};

#endif /* STEPPER_H_ */
