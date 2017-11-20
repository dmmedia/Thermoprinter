/*
 * macros.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef MACROS_H_
#define MACROS_H_

#include <math.h>

#include "SREGEmulation.h"
#include <stm32l0xx_hal_gpio.h>

#define FORCE_INLINE __attribute__((always_inline)) inline

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

#define sq(x) ((x)*(x))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define MAX3(a, b, c)       max(max(a, b), c)
#define MAX4(a, b, c, d)    max(MAX3(a, b, c), d)

#define PIN_EXISTS(IO) IS_GPIO_PIN_AVAILABLE(IO ## _PORT, IO ## _PIN)

#define NOOP do{} while(0)

void setInput(GPIO_TypeDef  *port, uint32_t pin, uint32_t mode = GPIO_MODE_INPUT) {
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = mode;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}  // set to input, which allows it to be pulled high by pullups

#define SET_INPUT(IO) setInput(IO ## _PORT, IO ## _PIN)
#define SET_INPUT_EXTI(IO) setInput(IO ## _PORT, IO ## _PIN, GPIO_MODE_IT_RISING_FALLING)

void setOutput(GPIO_TypeDef  *port, uint32_t pin) {
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(port, &GPIO_InitStruct);
}  // set to output

#define SET_OUTPUT(IO) setOutput(IO ## _PORT, IO ## _PIN)

// Bracket code that shouldn't be interrupted
#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG; cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif

#define WRITE(IO,V) HAL_GPIO_WritePin(IO ## _PORT, IO ## _PIN, V)
#define OUT_WRITE(IO, v) do{ SET_OUTPUT(IO); WRITE(IO, v); }while(0)

#define READ(IO) HAL_GPIO_ReadPin(IO ## _PORT, IO ## _PIN)

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

#endif /* MACROS_H_ */
