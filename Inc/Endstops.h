/*
 * Endstops.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef ENDSTOPS_H_
#define ENDSTOPS_H_

#include "main.h"
#include "Conditionals.h"

volatile uint8_t e_hit = 0; // Different from 0 when the endstops should be tested in detail.
                            // Must be reset to 0 by the test function when finished.

// This is what is really done inside the interrupts.
FORCE_INLINE void endstop_ISR_worker( void ) {
  e_hit = 2; // Because the detection of a e-stop hit has a 1 step debouncer it has to be called at least twice.
}

// Use one Routine to handle each group
// One ISR for all EXT-Interrupts
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	endstop_ISR_worker();
}


class Endstops {
public:
    static bool enabled, enabled_globally;
    static volatile char endstop_hit_bits; // use EndstopEnum as BIT value
    static byte current_endstop_bits, old_endstop_bits;

    Endstops() {}
	virtual ~Endstops();

    /**
     * Update the endstops bits from the pins
     */
    static void update();

    // Enable / disable endstop checking
    static void enable(bool onoff=true) { enabled = onoff; }

    // Clear endstops (i.e., they were hit intentionally) to suppress the report
    static void hit_on_purpose() { endstop_hit_bits = 0; }

    // Disable / Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    static void not_homing() { enabled = enabled_globally; }

    // Enable / disable endstop checking globally
    static void enable_globally(bool onoff=true) { enabled_globally = enabled = onoff; }

    /**
     * Initialize the endstop pins
     */
    void init();

};

#endif /* ENDSTOPS_H_ */
