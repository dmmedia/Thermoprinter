#pragma once

/*
 * MarlinSerial.h
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

namespace MarlinSerial {
	//
	// Public definitions and constants
	//

	#define DEC 10
	#define HEX 16
	#define OCT 8
	#define BIN 2
	#define BYTE 0

	//
	// Public variables
	//

	//
	// Public functions
	//

	void begin(uint32_t const);
    void end();
    void flush(void);
    uint8_t available(void);
    int32_t read(void);
    void write(uint8_t const c);
    void print(char const* str);
    void print(int32_t const , int32_t const = DEC);
    void print(uint32_t const, int32_t const = DEC);
    void print(double, int = 2);

    // End

}

extern "C" {
	void USART2_IRQHandler(void);
	void RNG_LPUART1_IRQHandler(void);
}
