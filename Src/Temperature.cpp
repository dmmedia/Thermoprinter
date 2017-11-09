/*
 * Temperature.cpp
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#include <Temperature.h>
#include "macros.h"

Temperature thermalManager;

// Init min and max temp with extreme values to prevent false errors during startup
int16_t Temperature::minttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_RAW_LO_TEMP , HEATER_1_RAW_LO_TEMP , HEATER_2_RAW_LO_TEMP, HEATER_3_RAW_LO_TEMP, HEATER_4_RAW_LO_TEMP),
        Temperature::maxttemp_raw[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_RAW_HI_TEMP , HEATER_1_RAW_HI_TEMP , HEATER_2_RAW_HI_TEMP, HEATER_3_RAW_HI_TEMP, HEATER_4_RAW_HI_TEMP),
        Temperature::minttemp[HOTENDS] = { 0 },
        Temperature::maxttemp[HOTENDS] = ARRAY_BY_HOTENDS1(16383);

/**
 * Initialize the temperature manager
 * The manager is implemented by periodic calls to manage_heater()
 */
void Temperature::init() {

  // Finish init of mult hotend arrays
  HOTEND_LOOP() maxttemp[e] = maxttemp[0];

  #ifdef DIDR2
    #define ANALOG_SELECT(pin) do{ if (pin < 8) SBI(DIDR0, pin); else SBI(DIDR2, pin - 8); }while(0)
  #else
    #define ANALOG_SELECT(pin) do{ SBI(DIDR0, pin); }while(0)
  #endif

  // Set analog inputs
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADIF) | 0x07;
  DIDR0 = 0;
  #ifdef DIDR2
    DIDR2 = 0;
  #endif
  #if HAS_TEMP_0
    ANALOG_SELECT(TEMP_0_PIN);
  #endif
  #if HAS_TEMP_1
    ANALOG_SELECT(TEMP_1_PIN);
  #endif
  #if HAS_TEMP_2
    ANALOG_SELECT(TEMP_2_PIN);
  #endif
  #if HAS_TEMP_3
    ANALOG_SELECT(TEMP_3_PIN);
  #endif
  #if HAS_TEMP_4
    ANALOG_SELECT(TEMP_4_PIN);
  #endif
  #if HAS_TEMP_BED
    ANALOG_SELECT(TEMP_BED_PIN);
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    ANALOG_SELECT(FILWIDTH_PIN);
  #endif

  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
  OCR0B = 128;
  SBI(TIMSK0, OCIE0B);

  // Wait for temperature measurement to settle
  delay(250);

  #define TEMP_MIN_ROUTINE(NR) \
    minttemp[NR] = HEATER_ ##NR## _MINTEMP; \
    while (analog2temp(minttemp_raw[NR], NR) < HEATER_ ##NR## _MINTEMP) { \
      if (HEATER_ ##NR## _RAW_LO_TEMP < HEATER_ ##NR## _RAW_HI_TEMP) \
        minttemp_raw[NR] += OVERSAMPLENR; \
      else \
        minttemp_raw[NR] -= OVERSAMPLENR; \
    }
  #define TEMP_MAX_ROUTINE(NR) \
    maxttemp[NR] = HEATER_ ##NR## _MAXTEMP; \
    while (analog2temp(maxttemp_raw[NR], NR) > HEATER_ ##NR## _MAXTEMP) { \
      if (HEATER_ ##NR## _RAW_LO_TEMP < HEATER_ ##NR## _RAW_HI_TEMP) \
        maxttemp_raw[NR] -= OVERSAMPLENR; \
      else \
        maxttemp_raw[NR] += OVERSAMPLENR; \
    }

  #ifdef HEATER_0_MINTEMP
    TEMP_MIN_ROUTINE(0);
  #endif
  #ifdef HEATER_0_MAXTEMP
    TEMP_MAX_ROUTINE(0);
  #endif
  #ifdef BED_MINTEMP
    while (analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_minttemp_raw += OVERSAMPLENR;
      #else
        bed_minttemp_raw -= OVERSAMPLENR;
      #endif
    }
  #endif // BED_MINTEMP
  #ifdef BED_MAXTEMP
    while (analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
      #if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
        bed_maxttemp_raw -= OVERSAMPLENR;
      #else
        bed_maxttemp_raw += OVERSAMPLENR;
      #endif
    }
  #endif // BED_MAXTEMP

  #if ENABLED(PROBING_HEATERS_OFF)
    paused = false;
  #endif
}
