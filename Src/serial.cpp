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
#include "SREGEmulation.h"
#include "typedefs.h"
#include "rcc.h"
#include "serial.h"

const char * const echomagic = "echo:";
const char * const errormagic = "Error:";

void serial_echopair_P(const char* s_P, const char *v)	{ serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, char v)         { serialprintPGM(s_P); SERIAL_CHAR(v); }
void serial_echopair_P(const char* s_P, int32_t v)      { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, float32_t v)    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, float64_t v)    { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, uint32_t v)     { serialprintPGM(s_P); SERIAL_ECHO(v); }



