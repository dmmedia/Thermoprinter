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
	bool enabled;
	bool enabled_globally;
	volatile int8_t endstop_hit_bits; // use EndstopEnum as BIT value
	uint8_t current_endstop_bits = 0U;
	uint8_t old_endstop_bits;

	volatile uint8_t e_hit = 0U; // Different from 0 when the endstops should be tested in detail.
								// Must be reset to 0 by the test function when finished.

    // Enable / disable endstop checking
    void enable(const bool onoff) {
    	enabled = onoff;
    }

    // Clear endstops (i.e., they were hit intentionally) to suppress the report
    void hit_on_purpose() {
    	endstop_hit_bits = 0;
    }

    // Disable / Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    void not_homing() {
    	enabled = enabled_globally;
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

	// Use one Routine to handle each group
	// One ISR for all EXT-Interrupts
	void GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
	{
	  Endstops::e_hit = 2U; // Because the detection of a e-stop hit has a 1 step debouncer it has to be called at least twice.
	  UNUSED(GPIO_Pin);
	}

}
