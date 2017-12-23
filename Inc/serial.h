/*
 * serial.h
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "MarlinSerial.h"
#define MYSERIAL customizedSerial

extern const char * const echomagic;

#define SERIAL_CHAR(x) (MarlinSerial::write(x))
#define SERIAL_EOL() SERIAL_CHAR(static_cast<uint8_t>('\n'))

#define SERIAL_PROTOCOL(x)                  (MYSERIAL.print(x))
#define SERIAL_PROTOCOLLNPGM(x)             (serialprintPGM(PSTR(x "\n")))
#define SERIAL_PROTOCOLLN(x)                do{ MYSERIAL.print(x); SERIAL_EOL(); }while(0)
#define SERIAL_PROTOCOLPAIR(name, value)    (serial_echopair_P((name),(value)))
#define SERIAL_PROTOCOLPGM(x)               (serialprintPGM(PSTR(x)))

#define SERIAL_ECHO_START()            (serialprintPGM(echomagic))
#define SERIAL_ECHO(x)                 SERIAL_PROTOCOL(x)
#define SERIAL_ECHOLN(x)               SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOPAIR(pre,value)     SERIAL_PROTOCOLPAIR(pre, value)
#define SERIAL_ECHOPGM(x)              SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLNPGM(x)            SERIAL_PROTOCOLLNPGM(x)

void serial_echopair_P(const char* s_P, const char *v);
void serial_echopair_P(const char* s_P, char v);
void serial_echopair_P(const char* s_P, int v);
void serial_echopair_P(const char* s_P, long v);
void serial_echopair_P(const char* s_P, float v);
void serial_echopair_P(const char* s_P, double v);
void serial_echopair_P(const char* s_P, unsigned int v);
void serial_echopair_P(const char* s_P, unsigned long v);
FORCE_INLINE void serial_echopair_P(const char* s_P, uint8_t v) { serial_echopair_P(s_P, (int)v); }
FORCE_INLINE void serial_echopair_P(const char* s_P, uint16_t v) { serial_echopair_P(s_P, (int)v); }
FORCE_INLINE void serial_echopair_P(const char* s_P, bool v) { serial_echopair_P(s_P, (int)v); }
FORCE_INLINE void serial_echopair_P(const char* s_P, void *v) { serial_echopair_P(s_P, (unsigned long)v); }

FORCE_INLINE void serialprintPGM(const char* str) {
  while (char ch = *(str++)) MYSERIAL.write(ch);
}

#endif /* SERIAL_H_ */
