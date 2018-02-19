#pragma once

/*
 * Stepper.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#include <stddef.h>
#include "typedefs.h"

namespace Stepper {
	//
	// Public constants and definitions
	//

	//
	// Public variables
	//

	extern Planner::block_t* current_block;  // A pointer to the block currently being traced

	//
	// Public functions
	//

	FORCE_INLINE void disable_MOTOR() {
		writePin(
			MOTOR_ENABLE_PORT,
			MOTOR_ENABLE_PIN,
			(
				(MOTOR_ENABLE_ON != GPIO::GPIO_PIN_RESET) ?
					GPIO::GPIO_PIN_RESET :
					GPIO::GPIO_PIN_SET
			)
		);
	}

	void enable_MOTOR();

	//
	// The stepper subsystem goes to sleep when it runs out of things to execute. Call this
	// to notify the subsystem that it is time to go to work.
	//
	void wake_up();

	//
	// Block until all buffered steps are executed
	//
	void synchronize();

	//
	// Initialize stepper hardware
	//
	void init();

	//
	// Handle a triggered endstop
	//
	void endstop_triggered();

	//
	// Set the current position in steps
	//
	void set_position(const long &a);

	// End

}
