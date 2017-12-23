//
// Endstops.cpp
//
//  Created on: 7. nov 2017
//      Author: Den
//

#include "Endstops.h"
#include "gpio.h"
#include "Configuration.h"
#include "macros.h"
#include "main.h"
#include "Planner.h"
#include "Stepper.h"

namespace Endstops {
	bool enabled, enabled_globally;
	volatile char endstop_hit_bits; // use EndstopEnum as BIT value
	uint8_t current_endstop_bits = 0U, old_endstop_bits;

	volatile uint8_t e_hit = 0U; // Different from 0 when the endstops should be tested in detail.
								// Must be reset to 0 by the test function when finished.

    // Enable / disable endstop checking
    void enable(bool onoff) {
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
    void enable_globally(bool onoff) {
    	enabled_globally = enabled = onoff;
    }
}

void Endstops::init() {
  setInput(MOTOR_FAULT_PORT, MOTOR_FAULT_PIN, GPIO_MODE_IT_RISING_FALLING);
  setInput(LO_BAT_PORT, LO_BAT_PIN, GPIO_MODE_IT_RISING_FALLING);
  setInput(VH_ON_CTRL_PORT, VH_ON_CTRL_PIN, GPIO_MODE_IT_RISING_FALLING);
  setInput(HEAD_UP_PORT, HEAD_UP_PIN, GPIO_MODE_IT_RISING_FALLING);
  setInput(PAPER_END_PORT, PAPER_END_PIN, GPIO_MODE_IT_RISING_FALLING);
  setInput(OVER_HEAT_PORT, OVER_HEAT_PIN, GPIO_MODE_IT_RISING_FALLING);

  // EXTI interrupt init
  NVIC_SetPriority(EXTI0_1_IRQn, 0U);
  NVIC_EnableIRQ(EXTI0_1_IRQn);

  NVIC_SetPriority(EXTI2_3_IRQn, 0U);
  NVIC_EnableIRQ(EXTI2_3_IRQn);

  NVIC_SetPriority(EXTI4_15_IRQn, 0U);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
} // Endstops::init

// Check endstops - Called from ISR!
void Endstops::update() {

  // TEST_ENDSTOP: test the old and the current status of an endstop
  #define TEST_ENDSTOP(ENDSTOP) (TEST(current_endstop_bits & old_endstop_bits, ENDSTOP))

  #define _ENDSTOP(END, STOP) END ##_## STOP
  #define _ENDSTOP_INVERTING(ES) ES ##_ENDSTOP_INVERTING
  #define _ENDSTOP_HIT(ES) SBI(endstop_hit_bits, ES)

  // UPDATE_ENDSTOP_BIT: set the current endstop bits for an endstop to its status
  #define UPDATE_ENDSTOP_BIT(ES) SET_BIT2(current_endstop_bits, ES, (READ(ES) != _ENDSTOP_INVERTING(ES)))

  #define UPDATE_ENDSTOP(END, STOP) do { \
      UPDATE_ENDSTOP_BIT(_ENDSTOP(END, STOP)); \
      if (TEST_ENDSTOP(_ENDSTOP(END, STOP)) && stepper.current_block->steps > 0) { \
        _ENDSTOP_HIT(_ENDSTOP(END, STOP)); \
        stepper.endstop_triggered(); \
      } \
    } while(0)

  /**
   * Check and update endstops according to conditions
   */

  UPDATE_ENDSTOP(MOTOR, FAULT);
  UPDATE_ENDSTOP(LO, BAT);
  UPDATE_ENDSTOP(VH_ON, CTRL);
  UPDATE_ENDSTOP(HEAD, UP);
  UPDATE_ENDSTOP(PAPER, END);
  UPDATE_ENDSTOP(OVER, HEAT);

  old_endstop_bits = current_endstop_bits;

} // Endstops::update()

// Use one Routine to handle each group
// One ISR for all EXT-Interrupts
void GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  Endstops::e_hit = 2U; // Because the detection of a e-stop hit has a 1 step debouncer it has to be called at least twice.
  UNUSED(GPIO_Pin);
}

