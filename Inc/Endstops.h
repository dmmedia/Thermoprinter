/*
 * Endstops.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef ENDSTOPS_H_
#define ENDSTOPS_H_

#include "main.h"
#include "Conditionals.h"

volatile uint8_t e_hit = 0; // Different from 0 when the endstops should be tested in detail.
                            // Must be reset to 0 by the test function when finished.

// This is what is really done inside the interrupts.
FORCE_INLINE void endstop_ISR_worker( void ) {
  e_hit = 2; // Because the detection of a e-stop hit has a 1 step debouncer it has to be called at least twice.
}

// Use one Routine to handle each group
// One ISR for all EXT-Interrupts
void endstop_ISR(void) { endstop_ISR_worker(); }

//#if defined(ARDUINO_AVR_MEGA2560) || defined(ARDUINO_AVR_MEGA)
  #undef  digitalPinToPCICR
  #define digitalPinToPCICR(p)    ( WITHIN(p, 10, 15) || \
                                    WITHIN(p, 50, 53) || \
                                    WITHIN(p, 62, 69) ? &PCICR : (uint8_t*)0 )
#undef  digitalPinToPCMSK
#define digitalPinToPCMSK(p)    ( WITHIN(p, 10, 13) || WITHIN(p, 50, 53) ? &PCMSK0 : \
                                  WITHIN(p, 14, 15) ? &PCMSK1 : \
                                  WITHIN(p, 62, 69) ? &PCMSK2 : \
                                  (uint8_t *)0 )
#undef  digitalPinToPCMSKbit
#define digitalPinToPCMSKbit(p) ( WITHIN(p, 10, 13) ? ((p) - 6) : \
                                  (p) == 14 || (p) == 51 ? 2 : \
                                  (p) == 15 || (p) == 52 ? 1 : \
                                  (p) == 50 ? 3 : \
                                  (p) == 53 ? 0 : \
                                  WITHIN(p, 62, 69) ? ((p) - 62) : \
                                  0 )
#undef  digitalPinToPCICRbit
#define digitalPinToPCICRbit(p) ( WITHIN(p, 10, 13) || WITHIN(p, 50, 53) ? 0 : \
                                  WITHIN(p, 14, 15) ? 1 : \
                                  WITHIN(p, 62, 69) ? 2 : \
                                  0 )
//#endif

// Install Pin change interrupt for a pin. Can be called multiple times.
void pciSetup(byte pin) {
  SBI(*digitalPinToPCMSK(pin), digitalPinToPCMSKbit(pin));  // enable pin
  SBI(PCIFR, digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  SBI(PCICR, digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

void setup_endstop_interrupts( void ) {

  #if HAS_X_MAX
    #if (digitalPinToInterrupt(X_MAX_PIN) != NOT_AN_INTERRUPT) // if pin has an external interrupt
      attachInterrupt(digitalPinToInterrupt(X_MAX_PIN), endstop_ISR, CHANGE); // assign it
    #else
      // Not all used endstop/probe -pins can raise interrupts. Please deactivate ENDSTOP_INTERRUPTS or change the pin configuration!
      static_assert(digitalPinToPCICR(X_MAX_PIN) != NULL, "X_MAX_PIN is not interrupt-capable"); // if pin has no pin change interrupt - error
      pciSetup(X_MAX_PIN);                                                            // assign it
    #endif
  #endif

  #if HAS_X_MIN
    #if (digitalPinToInterrupt(X_MIN_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(X_MIN_PIN), endstop_ISR, CHANGE);
    #else
      // Not all used endstop/probe -pins can raise interrupts. Please deactivate ENDSTOP_INTERRUPTS or change the pin configuration!
      static_assert(digitalPinToPCICR(X_MIN_PIN) != NULL, "X_MIN_PIN is not interrupt-capable");
      pciSetup(X_MIN_PIN);
    #endif
  #endif

  #if HAS_Y_MAX
    #if (digitalPinToInterrupt(Y_MAX_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Y_MAX_PIN), endstop_ISR, CHANGE);
    #else
      // Not all used endstop/probe -pins can raise interrupts. Please deactivate ENDSTOP_INTERRUPTS or change the pin configuration!
      static_assert(digitalPinToPCICR(Y_MAX_PIN) != NULL, "Y_MAX_PIN is not interrupt-capable");
      pciSetup(Y_MAX_PIN);
    #endif
  #endif

  #if HAS_Y_MIN
    #if (digitalPinToInterrupt(Y_MIN_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Y_MIN_PIN), endstop_ISR, CHANGE);
    #else
      // Not all used endstop/probe -pins can raise interrupts. Please deactivate ENDSTOP_INTERRUPTS or change the pin configuration!
      static_assert(digitalPinToPCICR(Y_MIN_PIN) != NULL, "Y_MIN_PIN is not interrupt-capable");
      pciSetup(Y_MIN_PIN);
    #endif
  #endif

  #if HAS_Z_MAX
    #if (digitalPinToInterrupt(Z_MAX_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Z_MAX_PIN), endstop_ISR, CHANGE);
    #else
      // Not all used endstop/probe -pins can raise interrupts. Please deactivate ENDSTOP_INTERRUPTS or change the pin configuration!
      static_assert(digitalPinToPCICR(Z_MAX_PIN) != NULL, "Z_MAX_PIN is not interrupt-capable");
      pciSetup(Z_MAX_PIN);
    #endif
  #endif

  #if HAS_Z_MIN
    #if (digitalPinToInterrupt(Z_MIN_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Z_MIN_PIN), endstop_ISR, CHANGE);
    #else
      // Not all used endstop/probe -pins can raise interrupts. Please deactivate ENDSTOP_INTERRUPTS or change the pin configuration!
      static_assert(digitalPinToPCICR(Z_MIN_PIN) != NULL, "Z_MIN_PIN is not interrupt-capable");
      pciSetup(Z_MIN_PIN);
    #endif
  #endif

  #if HAS_Z2_MAX
    #if (digitalPinToInterrupt(Z2_MAX_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Z2_MAX_PIN), endstop_ISR, CHANGE);
    #else
      // Not all used endstop/probe -pins can raise interrupts. Please deactivate ENDSTOP_INTERRUPTS or change the pin configuration!
      static_assert(digitalPinToPCICR(Z2_MAX_PIN) != NULL, "Z2_MAX_PIN is not interrupt-capable");
      pciSetup(Z2_MAX_PIN);
    #endif
  #endif

  #if HAS_Z2_MIN
    #if (digitalPinToInterrupt(Z2_MIN_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Z2_MIN_PIN), endstop_ISR, CHANGE);
    #else
      // Not all used endstop/probe -pins can raise interrupts. Please deactivate ENDSTOP_INTERRUPTS or change the pin configuration!
      static_assert(digitalPinToPCICR(Z2_MIN_PIN) != NULL, "Z2_MIN_PIN is not interrupt-capable");
      pciSetup(Z2_MIN_PIN);
    #endif
  #endif

  #if HAS_Z_MIN_PROBE_PIN
    #if (digitalPinToInterrupt(Z_MIN_PROBE_PIN) != NOT_AN_INTERRUPT)
      attachInterrupt(digitalPinToInterrupt(Z_MIN_PROBE_PIN), endstop_ISR, CHANGE);
    #else
      // Not all used endstop/probe -pins can raise interrupts. Please deactivate ENDSTOP_INTERRUPTS or change the pin configuration!
      static_assert(digitalPinToPCICR(Z_MIN_PROBE_PIN) != NULL, "Z_MIN_PROBE_PIN is not interrupt-capable");
      pciSetup(Z_MIN_PROBE_PIN);
    #endif
  #endif

  // If we arrive here without raising an assertion, each pin has either an EXT-interrupt or a PCI.
}

class Endstops {
public:
    static bool enabled, enabled_globally;
    static volatile char endstop_hit_bits; // use X_MIN, Y_MIN, Z_MIN and Z_MIN_PROBE as BIT value

    Endstops() {}
	virtual ~Endstops();

    // Enable / disable endstop checking
    static void enable(bool onoff=true) { enabled = onoff; }

    // Clear endstops (i.e., they were hit intentionally) to suppress the report
    static void hit_on_purpose() { endstop_hit_bits = 0; }

    // Disable / Enable endstops based on ENSTOPS_ONLY_FOR_HOMING and global enable
    static void not_homing() { enabled = enabled_globally; }

    // Enable / disable endstop checking globally
    static void enable_globally(bool onoff=true) { enabled_globally = enabled = onoff; }

    /**
     * Initialize the endstop pins
     */
    void init();

};

#endif /* ENDSTOPS_H_ */
