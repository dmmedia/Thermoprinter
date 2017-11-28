/*
 * Endstops.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef ENDSTOPS_H_
#define ENDSTOPS_H_

#include <stdint.h>

class Endstops {
public:
    static bool enabled, enabled_globally;
    static volatile char endstop_hit_bits; // use EndstopEnum as BIT value
    static uint8_t current_endstop_bits, old_endstop_bits;

    Endstops() {}

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

extern Endstops endstops;

#endif /* ENDSTOPS_H_ */
