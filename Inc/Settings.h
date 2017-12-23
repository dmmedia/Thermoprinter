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

    static void reset(void);

#if defined(EEPROM_SETTINGS)
  static bool load();

#else
  FORCE_INLINE static bool load() { reset(); return true; }
#endif

private:
  static void postprocess();

};

extern Settings settings;

#endif /* SETTINGS_H_ */
