/*
 * Settings.cpp
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#include <stdint.h>
#include <stm32l0xx.h>
#include "macros.h"
#include "Settings.h"
#include "typedefs.h"
#include "main.h"
#include "Planner.h"
#include "gpio.h"
#include "Configuration.h"
#include "Stepper.h"
#include "Endstops.h"

Settings settings;

#if defined(EEPROM_SETTINGS)
/**
 * M501 - Retrieve Configuration
 */
bool Settings::load() {
  uint16_t working_crc = 0;

  EEPROM_START();

  char stored_ver[4];
  EEPROM_READ(stored_ver);

  uint16_t stored_crc;
  EEPROM_READ(stored_crc);

  // Version has to match or defaults are used
  if (strncmp(version, stored_ver, 3) != 0) {
    if (stored_ver[0] != 'V') {
      stored_ver[0] = '?';
      stored_ver[1] = '\0';
    }
    reset();
  }
  else {
    float dummy = 0;
    bool dummyb;

    working_crc = 0; //clear before reading first "real data"

    // Number of esteppers may change
    uint8_t esteppers;
    EEPROM_READ(esteppers);

    // Get only the number of E stepper parameters previously stored
    // Any steppers added later are set to their defaults
    const float def1[] = DEFAULT_AXIS_STEPS_PER_UNIT, def2[] = DEFAULT_MAX_FEEDRATE;
    const uint32_t def3[] = DEFAULT_MAX_ACCELERATION;
    float tmp1[XYZ + esteppers], tmp2[XYZ + esteppers];
    uint32_t tmp3[XYZ + esteppers];
    EEPROM_READ(tmp1);
    EEPROM_READ(tmp2);
    EEPROM_READ(tmp3);
    LOOP_XYZE_N(i) {
      planner.axis_steps_per_mm[i]          = i < XYZ + esteppers ? tmp1[i] : def1[i < COUNT(def1) ? i : COUNT(def1) - 1];
      planner.max_feedrate_mm_s[i]          = i < XYZ + esteppers ? tmp2[i] : def2[i < COUNT(def2) ? i : COUNT(def2) - 1];
      planner.max_acceleration_mm_per_s2[i] = i < XYZ + esteppers ? tmp3[i] : def3[i < COUNT(def3) ? i : COUNT(def3) - 1];
    }

    EEPROM_READ(planner.acceleration);
    EEPROM_READ(planner.retract_acceleration);
    EEPROM_READ(planner.travel_acceleration);
    EEPROM_READ(planner.min_feedrate_mm_s);
    EEPROM_READ(planner.min_travel_feedrate_mm_s);
    EEPROM_READ(planner.min_segment_time);
    EEPROM_READ(planner.max_jerk);

    uint16_t val;
    for (uint8_t q = 0; q < 11; q++) EEPROM_READ(val);

    if (working_crc == stored_crc) {
      postprocess();
    }
    else {
      reset();
    }
  }

  #if ENABLED(EEPROM_CHITCHAT) && DISABLED(DISABLE_M503)
    report();
  #endif

  return !eeprom_error;
}

#endif // !EEPROM_SETTINGS

/**
 * M502 - Reset Configuration
 */
void Settings::reset(void) {
  planner.axis_steps_per_mm          = DEFAULT_STEPS_PER_UNIT;
  planner.max_feedrate_mm_s          = DEFAULT_MAX_FEEDRATE;
  planner.max_acceleration_mm_per_s2 = DEFAULT_MAX_ACCELERATION;

  planner.acceleration = DEFAULT_ACCELERATION;
  planner.travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  planner.min_feedrate_mm_s = DEFAULT_MINIMUMFEEDRATE;
  planner.min_segment_time = DEFAULT_MINSEGMENTTIME;
  planner.min_travel_feedrate_mm_s = DEFAULT_MINTRAVELFEEDRATE;
  planner.max_jerk = DEFAULT_JERK;

  Endstops::enable_globally(true);

  postprocess();
}

/**
 * Post-process after Retrieve or Reset
 */
void Settings::postprocess() {
  // steps per s2 needs to be updated to agree with units per s2
  planner.reset_acceleration_rates();

  // Refresh steps_to_mm with the reciprocal of axis_steps_per_mm
  // and init stepper.count, planner.position with current_position
  planner.refresh_positioning();
}
