/*
 * SREGEmulation.h
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#ifndef SREGEMULATION_H_
#define SREGEMULATION_H_

#include <stm32l053xx.h>
#include <cmsis_gcc.h>
#include <sys/cdefs.h>

/**
 * Re-enable interrupts.
 *
 * Call this after noInterrupts() to re-enable interrupt handling,
 * after you have finished with a timing-critical section of code.
 *
 * @see noInterrupts()
 */
static FORCE_INLINE void interrupts() {
	__enable_irq();
}

/**
 * Disable interrupts.
 *
 * After calling this function, all user-programmable interrupts will
 * be disabled.  You can call this function before a timing-critical
 * section of code, then call interrupts() to re-enable interrupt
 * handling.
 *
 * @see interrupts()
 */
static FORCE_INLINE void noInterrupts() {
	__disable_irq();
}

class SREGemulation
{
public:
	operator int () const __attribute__((always_inline)) {
		uint32_t primask { };
		asm volatile("mrs %0, primask\n" : "=r" (primask)::);
		if (primask) return 0;
		return (1<<7);
	}
	inline SREGemulation & operator = (int val) __attribute__((always_inline)) {
		if (val & (1<<7)) {
			interrupts();
		} else {
			noInterrupts();
		}
		return *this;
	}
};
extern SREGemulation SREG;

inline unsigned char  digitalPinToInterrupt(unsigned char Interrupt_pin) { return Interrupt_pin; }

#define sei() interrupts();
#define cli() noInterrupts();

#endif /* SREGEMULATION_H_ */
