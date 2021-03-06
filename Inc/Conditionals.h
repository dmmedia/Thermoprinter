#pragma once

/*
 * Conditionals.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#define MOTOR_ENABLE_INIT SET_OUTPUT(MOTOR_ENABLE)
#define MOTOR_ENABLE_WRITE(STATE)   HAL_GPIO_WritePin(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN, STATE)
#define MOTOR_ENABLE_READ READ(MOTOR_ENABLE_PIN)

// Stepper
#define HAS_MOTOR_ENABLE      (PIN_EXISTS(MOTOR_ENABLE))
#define HAS_MOTOR_DIR         (PIN_EXISTS(MOTOR_DIR))
#define HAS_MOTOR_STEP        (PIN_EXISTS(MOTOR_STEP))

// Endstops
#define HAS_MOTOR_FAULT (PIN_EXISTS(MOTOR_FAULT))
#define HAS_OVER_HEAT (PIN_EXISTS(OVER_HEAT))
#define HAS_LO_BAT (PIN_EXISTS(LO_BAT))
#define HAS_VH_ON_CTRL (PIN_EXISTS(VH_ON_CTRL))
#define HAS_PAPER_END (PIN_EXISTS(PAPER_END))
#define HAS_HEAD_UP (PIN_EXISTS(HEAD_UP))

#define HAS_STEPPER_RESET (PIN_EXISTS(STEPPER_RESET))
// Power Signal Control Definitions
// By default use ATX definition
#ifndef POWER_SUPPLY
	#define POWER_SUPPLY 0
	#define PS_ON_AWAKE		GPIO::GPIO_PIN_RESET
#endif
#if (POWER_SUPPLY == 1)     // 1 = ATX
  #define PS_ON_AWAKE  GPIO_PIN_RESET
  #define PS_ON_ASLEEP GPIO_PIN_SET
#elif (POWER_SUPPLY == 2)   // 2 = X-Box 360 203W
  #define PS_ON_AWAKE  GPIO_PIN_SET
  #define PS_ON_ASLEEP GPIO_PIN_RESET
#endif
constexpr bool hasPowerSwitch(void) {
	return ((POWER_SUPPLY > 0) && GPIO::pinExists(PS_ON_PORT, PS_ON_PIN));
}

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

#define THERMISTORHEATER_0 TEMP_SENSOR_0
#define HEATER_0_USES_THERMISTOR
