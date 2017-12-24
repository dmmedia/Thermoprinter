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
    extern volatile int8_t endstop_hit_bits; // use EndstopEnum as BIT value
    extern uint8_t current_endstop_bits, old_endstop_bits;

    extern volatile uint8_t e_hit; // Different from 0 when the endstops should be tested in detail.
                                // Must be reset to 0 by the test function when finished.
    /**
     * Update the endstops bits from the pins
     */
    void update();

    // Enable / disable endstop checking
    void enable(const bool onoff = true);

    // Clear endstops (i.e., they were hit intentionally) to suppress the report
    void hit_on_purpose();

    // Disable / Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    void not_homing();

    // Enable / disable endstop checking globally
    void enable_globally(const bool onoff = true);

    /**
     * Initialize the endstop pins
     */
    void init();

    FORCE_INLINE bool getEndstopInverting(EndstopEnum es) {
    	bool res = false;
    	switch (es) {
    	case MOTOR_FAULT:
    		res = MOTOR_FAULT_ENDSTOP_INVERTING;
    		break;
    	case LO_BAT:
    		res = LO_BAT_ENDSTOP_INVERTING;
    		break;
    	case VH_ON_CTRL:
    		res = VH_ON_CTRL_ENDSTOP_INVERTING;
    		break;
    	case HEAD_UP:
    		res = HEAD_UP_ENDSTOP_INVERTING;
    		break;
    	case PAPER_END:
    		res = PAPER_END_ENDSTOP_INVERTING;
    		break;
    	case OVER_HEAT:
    		res = OVER_HEAT_ENDSTOP_INVERTING;
    		break;
    	default:
    		// should not get here
    		break;
    	}
    	return res;
    }

    FORCE_INLINE uint32_t getPinByEndstop(EndstopEnum es) {
    	uint32_t res = 0U;
    	switch (es) {
    	case MOTOR_FAULT:
    		res = MOTOR_FAULT_PIN;
    		break;
    	case LO_BAT:
    		res = LO_BAT_PIN;
    		break;
    	case VH_ON_CTRL:
    		res = VH_ON_CTRL_PIN;
    		break;
    	case HEAD_UP:
    		res = HEAD_UP_PIN;
    		break;
    	case PAPER_END:
    		res = PAPER_END_PIN;
    		break;
    	case OVER_HEAT:
    		res = OVER_HEAT_PIN;
    		break;
    	default:
    		// should not get here
    		break;
    	}
    	return res;
    }

    FORCE_INLINE GPIO_TypeDef* getPortByEndstop(EndstopEnum es) {
    	GPIO_TypeDef* res = nullptr;
    	switch (es) {
    	case MOTOR_FAULT:
    		res = MOTOR_FAULT_PORT;
    		break;
    	case LO_BAT:
    		res = LO_BAT_PORT;
    		break;
    	case VH_ON_CTRL:
    		res = VH_ON_CTRL_PORT;
    		break;
    	case HEAD_UP:
    		res = HEAD_UP_PORT;
    		break;
    	case PAPER_END:
    		res = PAPER_END_PORT;
    		break;
    	case OVER_HEAT:
    		res = OVER_HEAT_PORT;
    		break;
    	default:
    		// should not get here
    		break;
    	}
    	return res;
    }

    FORCE_INLINE void updateEndstop(EndstopEnum es) {
		SET_BIT2(current_endstop_bits, es, (GPIO::read(getPortByEndstop(es), getPinByEndstop(es)) != getEndstopInverting(es)));
		if ((TEST(current_endstop_bits & old_endstop_bits, es)) && stepper.current_block->steps > 0) {
			SBI(endstop_hit_bits, es);
			stepper.endstop_triggered();
		}
	}

	// Use one Routine to handle each group
	// One ISR for all EXT-Interrupts
	void GPIO_EXTI_Callback(const uint16_t GPIO_Pin);

}

#endif /* ENDSTOPS_H_ */
