/*
 * Settings.h
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#ifndef SETTINGS_H_
#define SETTINGS_H_

class Settings {
public:
	Settings() {}
	virtual ~Settings() {}

    static void reset();

#if ENABLED(EEPROM_SETTINGS)
  static bool load();

#else
  __attribute__((always_inline)) inline static bool load() { reset(); return true; }
#endif

private:
  static void postprocess();

};

#endif /* SETTINGS_H_ */
