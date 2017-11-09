/*
 * Conditionals.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef CONDITIONALS_H_
#define CONDITIONALS_H_

#include "macros.h"

  /**
   * DISTINCT_E_FACTORS affects how some E factors are accessed
   */
#undef DISTINCT_E_FACTORS
#define XYZE_N XYZE
#define E_AXIS_N E_AXIS

  /**
   * ARRAY_BY_EXTRUDERS based on EXTRUDERS
   */
#define ARRAY_BY_EXTRUDERS(...) ARRAY_N(EXTRUDERS, __VA_ARGS__)
#define ARRAY_BY_EXTRUDERS1(v1) ARRAY_BY_EXTRUDERS(v1, v1, v1, v1, v1, v1)

#define X_ENABLE_INIT SET_OUTPUT(X_ENABLE_PIN)
#define X_ENABLE_WRITE(STATE) WRITE(X_ENABLE_PIN,STATE)
#define X_ENABLE_READ READ(X_ENABLE_PIN)

#define X_DIR_INIT SET_OUTPUT(X_DIR_PIN)
#define X_DIR_WRITE(STATE) WRITE(X_DIR_PIN,STATE)
#define X_DIR_READ READ(X_DIR_PIN)

#define X_STEP_INIT SET_OUTPUT(X_STEP_PIN)
#define X_STEP_WRITE(STATE) WRITE(X_STEP_PIN,STATE)
#define X_STEP_READ READ(X_STEP_PIN)

#define Y_ENABLE_INIT SET_OUTPUT(Y_ENABLE_PIN)
#define Y_ENABLE_WRITE(STATE) WRITE(Y_ENABLE_PIN,STATE)
#define Y_ENABLE_READ READ(Y_ENABLE_PIN)

#define Y_DIR_INIT SET_OUTPUT(Y_DIR_PIN)
#define Y_DIR_WRITE(STATE) WRITE(Y_DIR_PIN,STATE)
#define Y_DIR_READ READ(Y_DIR_PIN)

#define Y_STEP_INIT SET_OUTPUT(Y_STEP_PIN)
#define Y_STEP_WRITE(STATE) WRITE(Y_STEP_PIN,STATE)
#define Y_STEP_READ READ(Y_STEP_PIN)

#define Z_ENABLE_INIT SET_OUTPUT(Z_ENABLE_PIN)
#define Z_ENABLE_WRITE(STATE) WRITE(Z_ENABLE_PIN,STATE)
#define Z_ENABLE_READ READ(Z_ENABLE_PIN)

#define Z_DIR_INIT SET_OUTPUT(Z_DIR_PIN)
#define Z_DIR_WRITE(STATE) WRITE(Z_DIR_PIN,STATE)
#define Z_DIR_READ READ(Z_DIR_PIN)

#define Z_STEP_INIT SET_OUTPUT(Z_STEP_PIN)
#define Z_STEP_WRITE(STATE) WRITE(Z_STEP_PIN,STATE)
#define Z_STEP_READ READ(Z_STEP_PIN)

#define E0_ENABLE_INIT SET_OUTPUT(E0_ENABLE_PIN)
#define E0_ENABLE_WRITE(STATE) WRITE(E0_ENABLE_PIN,STATE)
#define E0_ENABLE_READ READ(E0_ENABLE_PIN)

#define E0_DIR_INIT SET_OUTPUT(E0_DIR_PIN)
#define E0_DIR_WRITE(STATE) WRITE(E0_DIR_PIN,STATE)
#define E0_DIR_READ READ(E0_DIR_PIN)

#define E0_STEP_INIT SET_OUTPUT(E0_STEP_PIN)
#define E0_STEP_WRITE(STATE) WRITE(E0_STEP_PIN,STATE)
#define E0_STEP_READ READ(E0_STEP_PIN)

// Steppers
  #define HAS_X_ENABLE      (PIN_EXISTS(X_ENABLE))
  #define HAS_X_DIR         (PIN_EXISTS(X_DIR))
  #define HAS_X_STEP        (PIN_EXISTS(X_STEP))
  #define HAS_X_MICROSTEPS  (PIN_EXISTS(X_MS1))

  #define HAS_Y_ENABLE      (PIN_EXISTS(Y_ENABLE))
  #define HAS_Y_DIR         (PIN_EXISTS(Y_DIR))
  #define HAS_Y_STEP        (PIN_EXISTS(Y_STEP))
  #define HAS_Z_MICROSTEPS  (PIN_EXISTS(Z_MS1))

  #define HAS_Z_ENABLE      (PIN_EXISTS(Z_ENABLE))
  #define HAS_Z_DIR         (PIN_EXISTS(Z_DIR))
  #define HAS_Z_STEP        (PIN_EXISTS(Z_STEP))

  // Extruder steppers and solenoids
  #define HAS_E0_ENABLE     (PIN_EXISTS(E0_ENABLE))
  #define HAS_E0_DIR        (PIN_EXISTS(E0_DIR))
  #define HAS_E0_STEP       (PIN_EXISTS(E0_STEP))
  #define HAS_E0_MICROSTEPS (PIN_EXISTS(E0_MS1))
  #define HAS_SOLENOID_0    (PIN_EXISTS(SOL0))

  // Endstops and bed probe
  #define HAS_X_MIN (PIN_EXISTS(X_MIN))
  #define HAS_X_MAX (PIN_EXISTS(X_MAX))
  #define HAS_Y_MIN (PIN_EXISTS(Y_MIN))
  #define HAS_Y_MAX (PIN_EXISTS(Y_MAX))
  #define HAS_Z_MIN (PIN_EXISTS(Z_MIN))
  #define HAS_Z_MAX (PIN_EXISTS(Z_MAX))
  #define HAS_Z2_MIN (PIN_EXISTS(Z2_MIN))
  #define HAS_Z2_MAX (PIN_EXISTS(Z2_MAX))

  /**
   * Set the home position based on settings or manual overrides
   */
  #define X_HOME_POS (X_HOME_DIR < 0 ? X_MIN_POS : X_MAX_POS)
  #define Y_HOME_POS (Y_HOME_DIR < 0 ? Y_MIN_POS : Y_MAX_POS)
  #define Z_HOME_POS (Z_HOME_DIR < 0 ? Z_MIN_POS : Z_MAX_POS)

  /**
   * Axis lengths and center
   */
  #define X_MAX_LENGTH (X_MAX_POS - (X_MIN_POS))
  #define Y_MAX_LENGTH (Y_MAX_POS - (Y_MIN_POS))
  #define Z_MAX_LENGTH (Z_MAX_POS - (Z_MIN_POS))

  #define HAS_STEPPER_RESET (PIN_EXISTS(STEPPER_RESET))
  // Power Signal Control Definitions
  // By default use ATX definition
  #ifndef POWER_SUPPLY
    #define POWER_SUPPLY 1
  #endif
  #if (POWER_SUPPLY == 1)     // 1 = ATX
    #define PS_ON_AWAKE  LOW
    #define PS_ON_ASLEEP HIGH
  #elif (POWER_SUPPLY == 2)   // 2 = X-Box 360 203W
    #define PS_ON_AWAKE  HIGH
    #define PS_ON_ASLEEP LOW
  #endif
  #define HAS_POWER_SWITCH (POWER_SUPPLY > 0 && PIN_EXISTS(PS_ON))

  /**
   * Extruders have some combination of stepper motors and hotends
   * so we separate these concepts into the defines:
   *
   *  EXTRUDERS    - Number of Selectable Tools
   *  HOTENDS      - Number of hotends, whether connected or separate
   *  E_STEPPERS   - Number of actual E stepper motors
   *  E_MANUAL     - Number of E steppers for LCD move options
   *  TOOL_E_INDEX - Index to use when getting/setting the tool state
   *
   */
  #define HOTENDS       EXTRUDERS

  /**
   * ARRAY_BY_HOTENDS based on HOTENDS
   */
  #define ARRAY_BY_HOTENDS(...) ARRAY_N(HOTENDS, __VA_ARGS__)
  #define ARRAY_BY_HOTENDS1(v1) ARRAY_BY_HOTENDS(v1, v1, v1, v1, v1, v1)

#endif /* CONDITIONALS_H_ */
