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

#if HAS_STB_EN
  #if ENABLED(ENDSTOPPULLUP_STBEN)
    SET_INPUT_PULLUP(STB_EN_PIN);
  #else
    SET_INPUT(STB_EN_PIN);
  #endif
#endif

  #if HAS_MOTOR_EN
    #if ENABLED(ENDSTOPPULLUP_MOTOREN)
      SET_INPUT_PULLUP(MOTOR_EN_PIN);
    #else
      SET_INPUT(MOTOR_EN_PIN);
    #endif
  #endif

  #if HAS_LOW_BAT
    #if ENABLED(ENDSTOPPULLUP_LOWBAT)
      SET_INPUT_PULLUP(LOW_BAT_PIN);
    #else
      SET_INPUT(LOW_ABT_PIN);
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

  #if ENABLED(G38_PROBE_TARGET) && PIN_EXISTS(Z_MIN_PROBE) && !(CORE_IS_XY || CORE_IS_XZ)
    // If G38 command is active check Z_MIN_PROBE for ALL movement
    if (G38_move) {
      UPDATE_ENDSTOP_BIT(Z, MIN_PROBE);
      if (TEST_ENDSTOP(_ENDSTOP(Z, MIN_PROBE))) {
        if      (stepper.current_block->steps[_AXIS(X)] > 0) { _ENDSTOP_HIT(X, MIN); stepper.endstop_triggered(_AXIS(X)); }
        else if (stepper.current_block->steps[_AXIS(Y)] > 0) { _ENDSTOP_HIT(Y, MIN); stepper.endstop_triggered(_AXIS(Y)); }
        else if (stepper.current_block->steps[_AXIS(Z)] > 0) { _ENDSTOP_HIT(Z, MIN); stepper.endstop_triggered(_AXIS(Z)); }
        G38_endstop_hit = true;
      }
    }
  #endif

  /**
   * Define conditions for checking endstops
   */

  #if IS_CORE
    #define S_(N) stepper.current_block->steps[CORE_AXIS_##N]
    #define D_(N) stepper.motor_direction(CORE_AXIS_##N)
  #endif

  #if CORE_IS_XY || CORE_IS_XZ
    /**
     * Head direction in -X axis for CoreXY and CoreXZ bots.
     *
     * If steps differ, both axes are moving.
     * If DeltaA == -DeltaB, the movement is only in the 2nd axis (Y or Z, handled below)
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X)
     */
    #if ENABLED(COREXY) || ENABLED(COREXZ)
      #define X_CMP ==
    #else
      #define X_CMP !=
    #endif
    #define X_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) X_CMP D_(2)) )
    #define X_AXIS_HEAD X_HEAD
  #else
    #define X_MOVE_TEST stepper.current_block->steps[X_AXIS] > 0
    #define X_AXIS_HEAD X_AXIS
  #endif

  #if CORE_IS_XY || CORE_IS_YZ
    /**
     * Head direction in -Y axis for CoreXY / CoreYZ bots.
     *
     * If steps differ, both axes are moving
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X or Y)
     * If DeltaA == -DeltaB, the movement is only in the 2nd axis (Y or Z)
     */
    #if ENABLED(COREYX) || ENABLED(COREYZ)
      #define Y_CMP ==
    #else
      #define Y_CMP !=
    #endif
    #define Y_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) Y_CMP D_(2)) )
    #define Y_AXIS_HEAD Y_HEAD
  #else
    #define Y_MOVE_TEST stepper.current_block->steps[Y_AXIS] > 0
    #define Y_AXIS_HEAD Y_AXIS
  #endif

  #if CORE_IS_XZ || CORE_IS_YZ
    /**
     * Head direction in -Z axis for CoreXZ or CoreYZ bots.
     *
     * If steps differ, both axes are moving
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X or Y, already handled above)
     * If DeltaA == -DeltaB, the movement is only in the 2nd axis (Z)
     */
    #if ENABLED(COREZX) || ENABLED(COREZY)
      #define Z_CMP ==
    #else
      #define Z_CMP !=
    #endif
    #define Z_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) Z_CMP D_(2)) )
    #define Z_AXIS_HEAD Z_HEAD
  #else
    #define Z_MOVE_TEST stepper.current_block->steps[Z_AXIS] > 0
    #define Z_AXIS_HEAD Z_AXIS
  #endif

  // With Dual X, endstops are only checked in the homing direction for the active extruder
  #if ENABLED(DUAL_X_CARRIAGE)
    #define E0_ACTIVE stepper.current_block->active_extruder == 0
    #define X_MIN_TEST ((X_HOME_DIR < 0 && E0_ACTIVE) || (X2_HOME_DIR < 0 && !E0_ACTIVE))
    #define X_MAX_TEST ((X_HOME_DIR > 0 && E0_ACTIVE) || (X2_HOME_DIR > 0 && !E0_ACTIVE))
  #else
    #define X_MIN_TEST true
    #define X_MAX_TEST true
  #endif

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
