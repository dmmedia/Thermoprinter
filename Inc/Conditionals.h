/*
 * Conditionals.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef CONDITIONALS_H_
#define CONDITIONALS_H_

#include "macros.h"

#define MOTOR_ENABLE_INIT SET_OUTPUT(MOTOR_ENABLE_PIN)
#define MOTOR_ENABLE_WRITE(STATE) WRITE(MOTOR_ENABLE_PIN,STATE)
#define MOTOR_ENABLE_READ READ(MOTOR_ENABLE_PIN)

#define MOTOR_DIR_INIT SET_OUTPUT(MOTOR_DIR_PIN)
#define MOTOR_DIR_WRITE(STATE) WRITE(MOTOR_DIR_PIN,STATE)
#define MOTOR_DIR_READ READ(MOTOR_DIR_PIN)

#define MOTOR_STEP_INIT SET_OUTPUT(MOTOR_STEP_PIN)
#define MOTOR_STEP_WRITE(STATE) WRITE(MOTOR_STEP_PIN,STATE)
#define MOTOR_STEP_READ READ(MOTOR_STEP_PIN)

// Stepper
#define HAS_MOTOR_ENABLE      (PIN_EXISTS(MOTOR_ENABLE))
#define HAS_MOTOR_DIR         (PIN_EXISTS(MOTOR_DIR))
#define HAS_MOTOR_STEP        (PIN_EXISTS(MOTOR_STEP))

// Endstops
  #define HAS_X_MIN (PIN_EXISTS(X_MIN))
  #define HAS_X_MAX (PIN_EXISTS(X_MAX))
  #define HAS_Y_MIN (PIN_EXISTS(Y_MIN))
  #define HAS_Y_MAX (PIN_EXISTS(Y_MAX))
  #define HAS_Z_MIN (PIN_EXISTS(Z_MIN))
  #define HAS_Z_MAX (PIN_EXISTS(Z_MAX))
  #define HAS_Z2_MIN (PIN_EXISTS(Z2_MIN))
  #define HAS_Z2_MAX (PIN_EXISTS(Z2_MAX))

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

  #define MAX_STEP_FREQUENCY 1000

  #define CYCLES_PER_MICROSECOND (SystemCoreClock / 1000000L) // 16 or 20

  // Stepper pulse duration, in cycles
  #define STEP_PULSE_CYCLES ((MINIMUM_STEPPER_PULSE) * CYCLES_PER_MICROSECOND)

  /**
   * Temp Sensor defines
   */
  #if TEMP_SENSOR_0 == 0
    #undef PRINTHEAD_MINTEMP
    #undef PRINTHEAD_MAXTEMP
  #elif TEMP_SENSOR_0 > 0
    #define THERMISTORHEATER_0 TEMP_SENSOR_0
    #define PRINTHEAD_USES_THERMISTOR
  #endif

  // Thermistors
  #define HAS_TEMP_0 (PIN_EXISTS(TEMP_0) && TEMP_SENSOR_0 != 0 && TEMP_SENSOR_0 > -2)

  // Sensors
  #define HAS_FILAMENT_WIDTH_SENSOR (PIN_EXISTS(FILWIDTH))

#endif /* CONDITIONALS_H_ */
