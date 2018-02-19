#pragma once

/*
 * macros.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#define FORCE_INLINE __attribute__((always_inline)) inline

#define UNUSED(x) ((void)(x))

#ifndef _BV
  #define _BV(PIN) (1UL << PIN)
#endif
#define TEST(n,b) (((n)&_BV(b))!=0U)
#define SBI(n,b) (n |= _BV(b))
#define CBI(n,b) (n &= ~_BV(b))
#define SET_BIT2(n,b,value) (n) ^= ((-value)^(n)) & (_BV(b))
FORCE_INLINE bool isBitSet(uint32_t reg, uint32_t bit) {
	return ((reg & bit) != RESET);
}
#define IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == RESET)

#define sq(x) ((x)*(x))
#define min(a,b) (((a)<(b))?(a):(b))
#define max(a,b) (((a)>(b))?(a):(b))
#define MAX3(a, b, c)       max(max(a, b), c)
#define MAX4(a, b, c, d)    max(MAX3(a, b, c), d)

#define NOOP asm("NOP");

#define LOW  0x0

// Highly granular delays for step pulses, etc.
#define DELAY_0_NOP NOOP
#define DELAY_1_NOP asm("NOP")
#define DELAY_2_NOP DELAY_1_NOP; DELAY_1_NOP
#define DELAY_3_NOP DELAY_1_NOP; DELAY_2_NOP
#define DELAY_4_NOP DELAY_1_NOP; DELAY_3_NOP
#define DELAY_5_NOP DELAY_1_NOP; DELAY_4_NOP

#define DELAY_NOPS(X) \
  switch (X) { \
    case 20: DELAY_1_NOP; case 19: DELAY_1_NOP; \
    case 18: DELAY_1_NOP; case 17: DELAY_1_NOP; \
    case 16: DELAY_1_NOP; case 15: DELAY_1_NOP; \
    case 14: DELAY_1_NOP; case 13: DELAY_1_NOP; \
    case 12: DELAY_1_NOP; case 11: DELAY_1_NOP; \
    case 10: DELAY_1_NOP; case 9:  DELAY_1_NOP; \
    case 8:  DELAY_1_NOP; case 7:  DELAY_1_NOP; \
    case 6:  DELAY_1_NOP; case 5:  DELAY_1_NOP; \
    case 4:  DELAY_1_NOP; case 3:  DELAY_1_NOP; \
    case 2:  DELAY_1_NOP; case 1:  DELAY_1_NOP; \
  }

#define DELAY_10_NOP DELAY_5_NOP;  DELAY_5_NOP
#define DELAY_20_NOP DELAY_10_NOP; DELAY_10_NOP

#define WITHIN(V,L,H) (((V) >= (L)) && ((V) <= (H)))
#define NUMERIC(a) WITHIN(a, '0', '9')
#define DECIMAL(a) (NUMERIC(a) || (a == '.'))
#define NUMERIC_SIGNED(a) (NUMERIC(a) || (a) == '-' || (a) == '+')
#define DECIMAL_SIGNED(a) (DECIMAL(a) || ((a) == '-') || ((a) == '+'))
#define COUNT(a) (sizeof(a)/sizeof(*a))
#define ZERO(a) memset(a,0,sizeof(a))
#define COPY(a,b) memcpy(a,b,min(sizeof(a),sizeof(b)))

// Macros to contrain values
FORCE_INLINE uint32_t NOLESS(uint32_t v, uint32_t n) {
	if (v < n) {
		return n;
	}
	return v;
}
#define NOMORE(v,n) do{ if (v > n) v = n; }while(0)
