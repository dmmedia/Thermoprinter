/*
 * Endstops.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef ENDSTOPS_H_
#define ENDSTOPS_H_

#include <stdint.h>

namespace Endstops {
    extern bool enabled, enabled_globally;
    extern volatile char endstop_hit_bits; // use EndstopEnum as BIT value
    extern uint8_t current_endstop_bits, old_endstop_bits;

    extern volatile uint8_t e_hit; // Different from 0 when the endstops should be tested in detail.
                                // Must be reset to 0 by the test function when finished.
    /**
     * Update the endstops bits from the pins
     */
    void update();

    // Enable / disable endstop checking
    void enable(bool onoff = true);

    // Clear endstops (i.e., they were hit intentionally) to suppress the report
    void hit_on_purpose();

    // Disable / Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    void not_homing();

    // Enable / disable endstop checking globally
    void enable_globally(bool onoff = true);

    /**
     * Initialize the endstop pins
     */
    void init();
}

#endif /* ENDSTOPS_H_ */
