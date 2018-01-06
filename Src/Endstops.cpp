//
// Endstops.cpp
//
//  Created on: 7. nov 2017
//      Author: Den
//

#include <stdint.h>
#include <stm32l0xx.h>
#include "macros.h"
#include "typedefs.h"
#include "main.h"
#include "gpio.h"
#include "Configuration.h"
#include "Planner.h"
#include "Stepper.h"
#include "Endstops.h"

namespace Endstops {
	//
	// Private definitions
	//

	//
	// Private variables
	//

	static bool enabled_globally = false;
	static volatile int8_t endstop_hit_bits = 0; // use EndstopEnum as BIT value
	static uint8_t current_endstop_bits = 0U;
	static uint8_t old_endstop_bits = 0U;

	//
	// Private function declarations
	//

    static FORCE_INLINE void updateEndstop(EndstopEnum es);

    //
	// Public variable initialization
	//

	bool enabled = false;

	volatile uint8_t e_hit = 0U; // Different from 0 when the endstops should be tested in detail.
								// Must be reset to 0 by the test function when finished.

	//
	// Namespace body
	//

    constexpr bool getEndstopInverting(EndstopEnum es) {
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

    constexpr uint32_t getPinByEndstop(EndstopEnum es) {
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

    constexpr GPIO_TypeDef* getPortByEndstop(EndstopEnum es) {
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

    // Enable / disable endstop checking
    void enable(const bool onoff) {
    	enabled = onoff;
    }

    // Enable / disable endstop checking globally
    void enable_globally(const bool onoff) {
    	enabled_globally = onoff;
    	enabled = onoff;
    }

    void init() {
		//lint -save -e1924 -e835 -e9078 -e923
		GPIO::setInput(MOTOR_FAULT_PORT, MOTOR_FAULT_PIN, GPIO::GPIO_MODE_IT_RISING_FALLING);
		GPIO::setInput(LO_BAT_PORT, LO_BAT_PIN, GPIO::GPIO_MODE_IT_RISING_FALLING);
		GPIO::setInput(VH_ON_CTRL_PORT, VH_ON_CTRL_PIN, GPIO::GPIO_MODE_IT_RISING_FALLING);
		GPIO::setInput(HEAD_UP_PORT, HEAD_UP_PIN, GPIO::GPIO_MODE_IT_RISING_FALLING);
		GPIO::setInput(PAPER_END_PORT, PAPER_END_PIN, GPIO::GPIO_MODE_IT_RISING_FALLING);
		GPIO::setInput(OVER_HEAT_PORT, OVER_HEAT_PIN, GPIO::GPIO_MODE_IT_RISING_FALLING);
		//lint -restore

		// EXTI interrupt init
		NVIC_SetPriority(EXTI0_1_IRQn, 0U);
		NVIC_EnableIRQ(EXTI0_1_IRQn);

		NVIC_SetPriority(EXTI2_3_IRQn, 0U);
		NVIC_EnableIRQ(EXTI2_3_IRQn);

		NVIC_SetPriority(EXTI4_15_IRQn, 0U);
		NVIC_EnableIRQ(EXTI4_15_IRQn);
	} // Endstops::init

    static FORCE_INLINE void updateEndstop(EndstopEnum es) {
		SET_BIT2(current_endstop_bits, es, (GPIO::read(getPortByEndstop(es), getPinByEndstop(es)) != getEndstopInverting(es)));
		if ((TEST(current_endstop_bits & old_endstop_bits, es)) && Stepper::current_block->steps > 0) {
			SBI(endstop_hit_bits, es);
			Stepper::endstop_triggered();
		}
	}

	// Check endstops - Called from ISR!
	void update() {
		//
		// Check and update endstops according to conditions
		//

		updateEndstop(MOTOR_FAULT);
		updateEndstop(LO_BAT);
		updateEndstop(VH_ON_CTRL);
		updateEndstop(HEAD_UP);
		updateEndstop(PAPER_END);
		updateEndstop(OVER_HEAT);

		old_endstop_bits = current_endstop_bits;

	} // Endstops::update()

	// End

}

extern "C" {
	// Use one Routine to handle each group
	// One ISR for all EXT-Interrupts
	void GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
	{
	  Endstops::e_hit = 2U; // Because the detection of a e-stop hit has a 1 step debouncer it has to be called at least twice.
	  UNUSED(GPIO_Pin);
	}

}
