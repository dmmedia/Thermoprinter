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

  #if HAS_Y_MIN
    #if ENABLED(ENDSTOPPULLUP_YMIN)
      SET_INPUT_PULLUP(Y_MIN_PIN);
    #else
      SET_INPUT(Y_MIN_PIN);
    #endif
  #endif

  #if HAS_Z_MIN
    #if ENABLED(ENDSTOPPULLUP_ZMIN)
      SET_INPUT_PULLUP(Z_MIN_PIN);
    #else
      SET_INPUT(Z_MIN_PIN);
    #endif
  #endif

  #if HAS_Z2_MIN
    #if ENABLED(ENDSTOPPULLUP_ZMIN)
      SET_INPUT_PULLUP(Z2_MIN_PIN);
    #else
      SET_INPUT(Z2_MIN_PIN);
    #endif
  #endif

  #if HAS_X_MAX
    #if ENABLED(ENDSTOPPULLUP_XMAX)
      SET_INPUT_PULLUP(X_MAX_PIN);
    #else
      SET_INPUT(X_MAX_PIN);
    #endif
  #endif

  #if HAS_Y_MAX
    #if ENABLED(ENDSTOPPULLUP_YMAX)
      SET_INPUT_PULLUP(Y_MAX_PIN);
    #else
      SET_INPUT(Y_MAX_PIN);
    #endif
  #endif

  #if HAS_Z_MAX
    #if ENABLED(ENDSTOPPULLUP_ZMAX)
      SET_INPUT_PULLUP(Z_MAX_PIN);
    #else
      SET_INPUT(Z_MAX_PIN);
    #endif
  #endif

  #if HAS_Z2_MAX
    #if ENABLED(ENDSTOPPULLUP_ZMAX)
      SET_INPUT_PULLUP(Z2_MAX_PIN);
    #else
      SET_INPUT(Z2_MAX_PIN);
    #endif
  #endif

  #if ENABLED(Z_MIN_PROBE_ENDSTOP)
    #if ENABLED(ENDSTOPPULLUP_ZMIN_PROBE)
      SET_INPUT_PULLUP(Z_MIN_PROBE_PIN);
    #else
      SET_INPUT(Z_MIN_PROBE_PIN);
    #endif
  #endif

} // Endstops::init

