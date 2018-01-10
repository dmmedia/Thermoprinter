#pragma once

/*
 * Settings.h
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

namespace Settings {
    void reset(void);

#if defined(EEPROM_SETTINGS)
    void load(void);
#else
  	FORCE_INLINE void load(void) {
  		reset();
  	}
#endif

  	void postprocess();
}

namespace RuntimeSettings {
	extern bool axis_relative_modes;
	extern bool relative_mode;

	extern float32_t feedrate_mm_s;

	extern int16_t feedrate_percentage;
	extern int16_t saved_feedrate_percentage;
}
