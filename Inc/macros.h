/*
 * macros.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef MACROS_H_
#define MACROS_H_

#define FORCE_INLINE __attribute__((always_inline)) inline

#define UNUSED(x) ((void)(x))

#define FABS(x)     fabs(x)
#define SQRT(x)     sqrt(x)
#define CEIL(x)     ceil(x)
#define FLOOR(x)    floor(x)
#define LROUND(x)   lround(x)

#ifndef _BV
  #define _BV(PIN) (1UL << PIN)
#endif
#define TEST(n,b) (((n)&_BV(b))!=0)
#define SBI(n,b) (n |= _BV(b))
#define CBI(n,b) (n &= ~_BV(b))
#define SET_BIT2(n,b,value) (n) ^= ((-value)^(n)) & (_BV(b))
#define SET_BIT(REG, BIT)     ((REG) |= (BIT))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(BIT))
#define IS_BIT_SET(REG, BIT)         (((REG) & (BIT)) != RESET)
#define IS_BIT_CLR(REG, BIT)         (((REG) & (BIT)) == RESET)
#define WRITE_REG(REG, VAL)   ((REG) = (VAL))
#define READ_REG(REG)         ((REG))
#define MODIFY_REG(REG, CLEARMASK, SETMASK)  WRITE_REG((REG), (((READ_REG(REG)) & (~(CLEARMASK))) | (SETMASK)))
#define READ_BIT(REG, BIT)    ((REG) & (BIT))

#define sq(x) ((x)*(x))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define MAX3(a, b, c)       max(max(a, b), c)
#define MAX4(a, b, c, d)    max(MAX3(a, b, c), d)

#define NOOP do{} while(0)

// Bracket code that shouldn't be interrupted
#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

#define LOW  0x0

// Highly granular delays for step pulses, etc.
#define DELAY_0_NOP NOOP
#define DELAY_1_NOP asm("nop");
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

#define _CAT(a, ...) a ## __VA_ARGS__
#define SWITCH_ENABLED_false 0
#define SWITCH_ENABLED_true  1
#define SWITCH_ENABLED_0     0
#define SWITCH_ENABLED_1     1
#define SWITCH_ENABLED_      1
#define ENABLED(b) _CAT(SWITCH_ENABLED_, b)
#define DISABLED(b) (!_CAT(SWITCH_ENABLED_, b))

#define WITHIN(V,L,H) (((V) >= (L)) && ((V) <= (H)))
#define NUMERIC(a) WITHIN(a, '0', '9')
#define DECIMAL(a) (NUMERIC(a) || (a == '.'))
#define NUMERIC_SIGNED(a) (NUMERIC(a) || (a) == '-' || (a) == '+')
#define DECIMAL_SIGNED(a) (DECIMAL(a) || ((a) == '-') || ((a) == '+'))
#define COUNT(a) (sizeof(a)/sizeof(*a))
#define ZERO(a) memset(a,0,sizeof(a))
#define COPY(a,b) memcpy(a,b,min(sizeof(a),sizeof(b)))

#define PSTR(s) (static_cast<const char *>(s))

// Macros to contrain values
#define NOLESS(v,n) do{ if (v < n) v = n; }while(0)
#define NOMORE(v,n) do{ if (v > n) v = n; }while(0)

#endif /* MACROS_H_ */
