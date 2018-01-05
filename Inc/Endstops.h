#pragma once

/*
 * Endstops.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#include <stdint.h>

namespace Endstops {
	//
	// Public definitions
	//

	enum EndstopEnum {
		MOTOR_FAULT,
		LO_BAT,
		VH_ON_CTRL,
		HEAD_UP,
		PAPER_END,
		OVER_HEAT
	};

	//
	// Public variables
	//

	extern bool enabled;

    extern volatile uint8_t e_hit; // Different from 0 when the endstops should be tested in detail.
                                // Must be reset to 0 by the test function when finished.

	//
	// Public functions
	//

	//
	// Update the endstops bits from the pins
	//
	void update();

    // Enable / disable endstop checking
    void enable(const bool onoff = true);

    // Enable / disable endstop checking globally
    void enable_globally(const bool onoff = true);

    //
    // Initialize the endstop pins
    //
    void init();

    // End
}

extern "C" {
	// Use one Routine to handle each group
	// One ISR for all EXT-Interrupts
	void GPIO_EXTI_Callback(const uint16_t GPIO_Pin);
}
