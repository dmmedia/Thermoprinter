/*
 * serial.cpp
 *
 *  Created on: 16. dets 2017
 *      Author: Den
 */

#include "macros.h"
#include "serial.h"

const char * const echomagic = "echo:";

void serial_echopair_P(const char* s_P, const char *v)   { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, char v)          { serialprintPGM(s_P); SERIAL_CHAR(v); }
void serial_echopair_P(const char* s_P, int v)           { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, long v)          { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, float v)         { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, double v)        { serialprintPGM(s_P); SERIAL_ECHO(v); }
void serial_echopair_P(const char* s_P, unsigned long v) { serialprintPGM(s_P); SERIAL_ECHO(v); }



