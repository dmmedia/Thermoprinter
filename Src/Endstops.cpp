/*
 * Endstops.cpp
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#include <Endstops.h>
#include "macros.h"
#include "Stepper.h"

Endstops endstops;

volatile char Endstops::endstop_hit_bits; // use MOTOR_FAULT, LOW_BAT, VH_ON_CTRL, HEAD_UP, PAPER_END and OVER_HEAT as BIT value

byte Endstops::current_endstop_bits = 0;

void Endstops::init() {
  if (HAS_MOTOR_FAULT) {
      SET_INPUT_EXTI(MOTOR_FAULT);
  }

  if (HAS_LO_BAT) {
      SET_INPUT_EXTI(LO_BAT);
  }

  if (HAS_VH_ON_CTRL) {
      SET_INPUT_EXTI(VH_ON_CTRL);
  }

  if (HAS_HEAD_UP) {
      SET_INPUT_EXTI(HEAD_UP);
  }

  if (HAS_PAPER_END) {
      SET_INPUT_EXTI(PAPER_END);
  }

  if (HAS_OVER_HEAT) {
      SET_INPUT_EXTI(OVER_HEAT);
  }

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
} // Endstops::init

// Check endstops - Called from ISR!
void Endstops::update() {

  // TEST_ENDSTOP: test the old and the current status of an endstop
  #define TEST_ENDSTOP(ENDSTOP) (TEST(current_endstop_bits & old_endstop_bits, ENDSTOP))

  #define _ENDSTOP(END, STOP) END ##_## STOP
  #define _ENDSTOP_PIN(ES) ES ##_PIN
  #define _ENDSTOP_INVERTING(ES) ES ##_ENDSTOP_INVERTING
  #define _ENDSTOP_HIT(ES) SBI(endstop_hit_bits, ES)

  // UPDATE_ENDSTOP_BIT: set the current endstop bits for an endstop to its status
  #define UPDATE_ENDSTOP_BIT(ES) SET_BIT2(current_endstop_bits, ES, (READ(ES) != _ENDSTOP_INVERTING(ES)))
  // COPY_BIT: copy the value of SRC_BIT to DST_BIT in DST
  #define COPY_BIT(DST, SRC_BIT, DST_BIT) SET_BIT2(DST, DST_BIT, TEST(DST, SRC_BIT))

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
