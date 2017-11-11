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

#include "SREGEmulation.h"
#include <stm32l0xx_hal_gpio.h>

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

#define PIN_EXISTS(IO) IS_GPIO_PIN_AVAILABLE(IO ## _PORT, IO ## _PIN)

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

// Compare Modes
typedef enum {
  COM_NORMAL,          //  0
  COM_TOGGLE,          //  1  Non-PWM: OCnx ... Both PWM (WGM 9,11,14,15): OCnA only ... else NORMAL
  COM_CLEAR_SET,       //  2  Non-PWM: OCnx ... Fast PWM: OCnx/Bottom ... PF-FC: OCnx Up/Down
  COM_SET_CLEAR        //  3  Non-PWM: OCnx ... Fast PWM: OCnx/Bottom ... PF-FC: OCnx Up/Down
} CompareMode;

// Clock Sources
typedef enum {
  CS_NONE,             //  0
  CS_PRESCALER_1,      //  1
  CS_PRESCALER_8,      //  2
  CS_PRESCALER_64,     //  3
  CS_PRESCALER_256,    //  4
  CS_PRESCALER_1024,   //  5
  CS_EXT_FALLING,      //  6
  CS_EXT_RISING        //  7
} ClockSource;

// Set Compare Mode bits
#define _SET_COM(T,Q,V) (TCCR##T##Q = (TCCR##T##Q & ~(0x3 << COM##T##Q##0)) | (int(V) << COM##T##Q##0))
#define SET_COM(T,Q,V) _SET_COM(T,Q,COM_##V)
#define SET_COMA(T,V) SET_COM(T,A,V)

// Set Clock Select bits
#define _SET_CS(T,V) (TCCR##T##B = (TCCR##T##B & ~(0x7 << CS##T##0)) | ((int(V) & 0x7) << CS##T##0))
#define _SET_CS0(V) _SET_CS(0,V)
#define _SET_CS1(V) _SET_CS(1,V)
#ifdef TCCR2
  #define _SET_CS2(V) (TCCR2 = (TCCR2 & ~(0x7 << CS20)) | (int(V) << CS20))
#else
  #define _SET_CS2(V) _SET_CS(2,V)
#endif
#define _SET_CS3(V) _SET_CS(3,V)
#define _SET_CS4(V) _SET_CS(4,V)
#define _SET_CS5(V) _SET_CS(5,V)
#define SET_CS0(V) _SET_CS0(CS_##V)
#define SET_CS1(V) _SET_CS1(CS_##V)
#ifdef TCCR2
  #define SET_CS2(V) _SET_CS2(CS2_##V)
#else
  #define SET_CS2(V) _SET_CS2(CS_##V)
#endif
#define SET_CS3(V) _SET_CS3(CS_##V)
#define SET_CS4(V) _SET_CS4(CS_##V)
#define SET_CS5(V) _SET_CS5(CS_##V)
#define SET_CS(T,V) SET_CS##T(V)

// Set Wave Generation Mode bits
#define _SET_WGM(T,V) do{ \
    TCCR##T##A = (TCCR##T##A & ~(0x3 << WGM##T##0)) | (( int(V)       & 0x3) << WGM##T##0); \
    TCCR##T##B = (TCCR##T##B & ~(0x3 << WGM##T##2)) | (((int(V) >> 2) & 0x3) << WGM##T##2); \
  }while(0)
#define SET_WGM(T,V) _SET_WGM(T,WGM_##V)

#endif /* MACROS_H_ */
