/*
 * Endstops.cpp
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#include <Endstops.h>

Endstops endstops;

volatile char Endstops::endstop_hit_bits; // use X_MIN, Y_MIN, Z_MIN and Z_MIN_PROBE as BIT value

void Endstops::init() {

  #if HAS_X_MIN
    #if ENABLED(ENDSTOPPULLUP_XMIN)
      SET_INPUT_PULLUP(X_MIN_PIN);
    #else
      SET_INPUT(X_MIN_PIN);
    #endif
  #endif

#if HAS_STB_EN
  #if ENABLED(ENDSTOPPULLUP_STBEN)
    SET_INPUT_PULLUP(STB_EN_PIN);
  #else
    SET_INPUT(STB_EN_PIN);
  #endif
#endif

  #if HAS_MOTOR_EN
    #if ENABLED(ENDSTOPPULLUP_MOTOREN)
      SET_INPUT_PULLUP(MOTOR_EN_PIN);
    #else
      SET_INPUT(MOTOR_EN_PIN);
    #endif
  #endif

  #if HAS_LOW_BAT
    #if ENABLED(ENDSTOPPULLUP_LOWBAT)
      SET_INPUT_PULLUP(LOW_BAT_PIN);
    #else
      SET_INPUT(LOW_ABT_PIN);
    #endif
  #endif

  #if HAS_VH_ON_CTRL
    #if ENABLED(ENDSTOPPULLUP_VHONCTRL)
      SET_INPUT_PULLUP(VH_ON_CTRL_PIN);
    #else
      SET_INPUT(VH_ON_CTRL_PIN);
    #endif
  #endif

  #if HAS_HEAD_UP
    #if ENABLED(ENDSTOPPULLUP_HEADUP)
      SET_INPUT_PULLUP(HEAD_UP_PIN);
    #else
      SET_INPUT(HEAD_UP_PIN);
    #endif
  #endif

  #if HAS_PAPER_END
    #if ENABLED(ENDSTOPPULLUP_PAPEREND)
      SET_INPUT_PULLUP(PAPER_END_PIN);
    #else
      SET_INPUT(PAPER_END_PIN);
    #endif
  #endif

  #if HAS_MOTOR_FAULT
    #if ENABLED(ENDSTOPPULLUP_MOTORFAULT)
      SET_INPUT_PULLUP(MOTOR_FAULT_PIN);
    #else
      SET_INPUT(MOTOR_FAULT_PIN);
    #endif
  #endif

  #if HAS_OVER_HEAT
    #if ENABLED(ENDSTOPPULLUP_OVERHEAT)
      SET_INPUT_PULLUP(OVER_HEAT_PIN);
    #else
      SET_INPUT(OVER_HEAT_PIN);
    #endif
  #endif

} // Endstops::init

