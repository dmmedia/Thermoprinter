/*
 * serial.cpp
 *
 *  Created on: 16. dets 2017
 *      Author: Den
 */

#include <stdint.h>
#include <stm32l0xx.h>
#include "macros.h"
#include "Configuration.h"
#include "typedefs.h"
#include "rcc.h"
#include "serial.h"

namespace Serial {
	//
	// Public variable initializations
	//

	const char * const echomagic = "echo:";
	const char * const errormagic = "Error:";

	//
	// Namespace body
	//

	void serial_echopair_P(const char* s_P, const char *v)	{ serialprintPGM(s_P); SERIAL_ECHO(v); }
	void serial_echopair_P(const char* s_P, char v)         { serialprintPGM(s_P); SERIAL_CHAR(v); }
	void serial_echopair_P(const char* s_P, int32_t v)      { serialprintPGM(s_P); SERIAL_ECHO(v); }
	void serial_echopair_P(const char* s_P, float32_t v)    { serialprintPGM(s_P); SERIAL_ECHO(v); }
	void serial_echopair_P(const char* s_P, float64_t v)    { serialprintPGM(s_P); SERIAL_ECHO(v); }
	void serial_echopair_P(const char* s_P, uint32_t v)     { serialprintPGM(s_P); SERIAL_ECHO(v); }

	void SERIAL_ECHO_START(void) {
		serialprintPGM(echomagic);
	}

	void SERIAL_ERROR_START(void) {
		serialprintPGM(errormagic);
	}

	void serialprintPGM(const char* str) {
		while (char ch = *(str++)) MarlinSerial::write(ch);
	}

	// End
}
