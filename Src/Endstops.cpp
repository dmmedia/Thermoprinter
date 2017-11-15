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

  #if HAS_MOTOR_FAULT
    #if ENABLED(ENDSTOPPULLUP_MOTORFAULT)
      SET_INPUT_PULLUP(MOTOR_FAULT_PIN);
    #else
      SET_INPUT(MOTOR_FAULT_PIN);
    #endif
  #endif

  #if HAS_LOW_BAT
    #if ENABLED(ENDSTOPPULLUP_LOWBAT)
      SET_INPUT_PULLUP(LOW_BAT_PIN);
    #else
      SET_INPUT(LOW_BAT_PIN);
    #endif
  #endif

  #if HAS_VH_ON_CTRL
    #if ENABLED(ENDSTOPPULLUP_VHONCTRL)
      SET_INPUT_PULLUP(VH_ON_CTRL_PIN);
    #else
      SET_INPUT(VH_ON_CTRL_PIN);
    #endif
  #endif

  #if HAS_HEAD_UP
    #if ENABLED(ENDSTOPPULLUP_HEADUP)
      SET_INPUT_PULLUP(HEAD_UP_PIN);
    #else
      SET_INPUT(HEAD_UP_PIN);
    #endif
  #endif

  #if HAS_PAPER_END
    #if ENABLED(ENDSTOPPULLUP_PAPEREND)
      SET_INPUT_PULLUP(PAPER_END_PIN);
    #else
      SET_INPUT(PAPER_END_PIN);
    #endif
  #endif

  #if HAS_MOTOR_FAULT
    #if ENABLED(ENDSTOPPULLUP_MOTORFAULT)
      SET_INPUT_PULLUP(MOTOR_FAULT_PIN);
    #else
      SET_INPUT(MOTOR_FAULT_PIN);
    #endif
  #endif

  #if HAS_OVER_HEAT
    #if ENABLED(ENDSTOPPULLUP_OVERHEAT)
      SET_INPUT_PULLUP(OVER_HEAT_PIN);
    #else
      SET_INPUT(OVER_HEAT_PIN);
    #endif
  #endif

} // Endstops::init

// Check endstops - Called from ISR!
void Endstops::update() {

  #define _ENDSTOP(AXIS, MINMAX) AXIS ##_## MINMAX
  #define _ENDSTOP_PIN(AXIS, MINMAX) AXIS ##_## MINMAX ##_PIN
  #define _ENDSTOP_INVERTING(AXIS, MINMAX) AXIS ##_## MINMAX ##_ENDSTOP_INVERTING
  #define _ENDSTOP_HIT(AXIS, MINMAX) SBI(endstop_hit_bits, _ENDSTOP(AXIS, MINMAX))

  // UPDATE_ENDSTOP_BIT: set the current endstop bits for an endstop to its status
  #define UPDATE_ENDSTOP_BIT(AXIS, MINMAX) SET_BIT(current_endstop_bits, _ENDSTOP(AXIS, MINMAX), (READ(_ENDSTOP_PIN(AXIS, MINMAX)) != _ENDSTOP_INVERTING(AXIS, MINMAX)))
  // COPY_BIT: copy the value of SRC_BIT to DST_BIT in DST
  #define COPY_BIT(DST, SRC_BIT, DST_BIT) SET_BIT(DST, DST_BIT, TEST(DST, SRC_BIT))

  #define UPDATE_ENDSTOP(AXIS,MINMAX) do { \
      UPDATE_ENDSTOP_BIT(AXIS, MINMAX); \
      if (TEST_ENDSTOP(_ENDSTOP(AXIS, MINMAX)) && stepper.current_block->steps[_AXIS(AXIS)] > 0) { \
        _ENDSTOP_HIT(AXIS, MINMAX); \
        stepper.endstop_triggered(_AXIS(AXIS)); \
      } \
    } while(0)

  /**
   * Define conditions for checking endstops
   */

  #define X_MOVE_TEST stepper.current_block->steps[X_AXIS] > 0
  #define X_AXIS_HEAD X_AXIS

  #define Y_MOVE_TEST stepper.current_block->steps[Y_AXIS] > 0
  #define Y_AXIS_HEAD Y_AXIS

  #define Z_MOVE_TEST stepper.current_block->steps[Z_AXIS] > 0
  #define Z_AXIS_HEAD Z_AXIS

  #define X_MIN_TEST true
  #define X_MAX_TEST true

  /**
   * Check and update endstops according to conditions
   */

  if (X_MOVE_TEST) {
    if (stepper.motor_direction(X_AXIS_HEAD)) {
      if (X_MIN_TEST) { // -direction
        #if HAS_X_MIN
          UPDATE_ENDSTOP(X, MIN);
        #endif
      }
    }
    else if (X_MAX_TEST) { // +direction
      #if HAS_X_MAX
        UPDATE_ENDSTOP(X, MAX);
      #endif
    }
  }

  if (Y_MOVE_TEST) {
    if (stepper.motor_direction(Y_AXIS_HEAD)) { // -direction
      #if HAS_Y_MIN
        UPDATE_ENDSTOP(Y, MIN);
      #endif
    }
    else { // +direction
      #if HAS_Y_MAX
        UPDATE_ENDSTOP(Y, MAX);
      #endif
    }
  }

  if (Z_MOVE_TEST) {
    if (stepper.motor_direction(Z_AXIS_HEAD)) { // Z -direction. Gantry down, bed up.
      #if HAS_Z_MIN
        #if ENABLED(Z_DUAL_ENDSTOPS)

          UPDATE_ENDSTOP_BIT(Z, MIN);
          #if HAS_Z2_MIN
            UPDATE_ENDSTOP_BIT(Z2, MIN);
          #else
            COPY_BIT(current_endstop_bits, Z_MIN, Z2_MIN);
          #endif

          test_dual_z_endstops(Z_MIN, Z2_MIN);

        #else // !Z_DUAL_ENDSTOPS

          #if ENABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)
            if (z_probe_enabled) UPDATE_ENDSTOP(Z, MIN);
          #else
            UPDATE_ENDSTOP(Z, MIN);
          #endif

        #endif // !Z_DUAL_ENDSTOPS

      #endif // HAS_Z_MIN

      // When closing the gap check the enabled probe
      #if ENABLED(Z_MIN_PROBE_ENDSTOP)
        if (z_probe_enabled) {
          UPDATE_ENDSTOP(Z, MIN_PROBE);
          if (TEST_ENDSTOP(Z_MIN_PROBE)) SBI(endstop_hit_bits, Z_MIN_PROBE);
        }
      #endif
    }
    else { // Z +direction. Gantry up, bed down.
      #if HAS_Z_MAX

        // Check both Z dual endstops
        #if ENABLED(Z_DUAL_ENDSTOPS)

          UPDATE_ENDSTOP_BIT(Z, MAX);
          #if HAS_Z2_MAX
            UPDATE_ENDSTOP_BIT(Z2, MAX);
          #else
            COPY_BIT(current_endstop_bits, Z_MAX, Z2_MAX);
          #endif

          test_dual_z_endstops(Z_MAX, Z2_MAX);

        // If this pin is not hijacked for the bed probe
        // then it belongs to the Z endstop
        #elif DISABLED(Z_MIN_PROBE_ENDSTOP) || Z_MAX_PIN != Z_MIN_PROBE_PIN

          UPDATE_ENDSTOP(Z, MAX);

        #endif // !Z_MIN_PROBE_PIN...
      #endif // Z_MAX_PIN
    }
  }

  old_endstop_bits = current_endstop_bits;

} // Endstops::update()
