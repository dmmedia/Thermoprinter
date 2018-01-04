/*
 * Settings.h
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

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
};

#endif /* SETTINGS_H_ */
