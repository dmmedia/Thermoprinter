/*
 * macros.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef MACROS_H_
#define MACROS_H_

#include <math.h>
#include <algorithm>

#define FORCE_INLINE __attribute__((always_inline)) inline

#define NUM_AXIS 4
#define XYZ  3
#define XYZE 4

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

// Macros for initializing arrays
#define ARRAY_6(v1, v2, v3, v4, v5, v6, ...) { v1, v2, v3, v4, v5, v6 }
#define ARRAY_5(v1, v2, v3, v4, v5, ...)     { v1, v2, v3, v4, v5 }
#define ARRAY_4(v1, v2, v3, v4, ...)         { v1, v2, v3, v4 }
#define ARRAY_3(v1, v2, v3, ...)             { v1, v2, v3 }
#define ARRAY_2(v1, v2, ...)                 { v1, v2 }
#define ARRAY_1(v1, ...)                     { v1 }

#define _ARRAY_N(N, ...) ARRAY_ ##N(__VA_ARGS__)
#define ARRAY_N(N, ...) _ARRAY_N(N, __VA_ARGS__)

#define sq(x) ((x)*(x))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define MAX3(a, b, c)       max(max(a, b), c)
#define MAX4(a, b, c, d)    max(MAX3(a, b, c), d)

#define PIN_EXISTS(PN) (defined(PN ##_PIN) && PN ##_PIN >= 0)

#define NOOP do{} while(0)

#define _SET_INPUT(IO) do {DIO ## IO ## _DDR &= ~_BV(DIO ## IO ## _PIN); } while (0)
#define SET_INPUT(IO) _SET_INPUT(IO)

#define _SET_OUTPUT(IO) do {DIO ## IO ## _DDR |= _BV(DIO ## IO ## _PIN); } while (0)
#define SET_OUTPUT(IO) _SET_OUTPUT(IO)

// Bracket code that shouldn't be interrupted
#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

#define _WRITE_NC(IO, v)  do { if (v) {DIO ##  IO ## _WPORT |= _BV(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~_BV(DIO ## IO ## _PIN); }; } while (0)

#define _WRITE_C(IO, v)   do { if (v) { \
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT |= _BV(DIO ## IO ## _PIN); } \
                                         CRITICAL_SECTION_END; \
                                       } \
                                       else { \
                                         CRITICAL_SECTION_START; \
                                         {DIO ##  IO ## _WPORT &= ~_BV(DIO ## IO ## _PIN); } \
                                         CRITICAL_SECTION_END; \
                                       } \
                                     } \
                                     while (0)
#define _WRITE(IO, v) do { if (&(DIO ## IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)
#define WRITE(IO,V) _WRITE(IO,V)
#define OUT_WRITE(IO, v) do{ SET_OUTPUT(IO); WRITE(IO, v); }while(0)

#define _READ(IO) ((bool)(DIO ## IO ## _RPORT & _BV(DIO ## IO ## _PIN)))
#define READ(IO) _READ(IO)

#define LOW  0x0

#endif /* MACROS_H_ */
