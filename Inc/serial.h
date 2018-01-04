/*
 * serial.h
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "MarlinSerial.h"

extern const char * const echomagic;
extern const char * const errormagic;

#define SERIAL_CHAR(x) (MarlinSerial::write(static_cast<uint8_t>(x)))
#define SERIAL_EOL() SERIAL_CHAR('\n')

#define SERIAL_PROTOCOL(x)                  (MarlinSerial::print(x))
#define SERIAL_PROTOCOLLNPGM(x)             (serialprintPGM(x "\n"))
#define SERIAL_PROTOCOLLN(x)                do{ MarlinSerial::print(x); SERIAL_EOL(); }while(0)
#define SERIAL_PROTOCOLPAIR(name, value)    (serial_echopair_P((name),(value)))
#define SERIAL_PROTOCOLPGM(x)               (serialprintPGM(x))

#define SERIAL_ECHO_START()            (serialprintPGM(echomagic))
#define SERIAL_ECHO(x)                 SERIAL_PROTOCOL(x)
#define SERIAL_ECHOLN(x)               SERIAL_PROTOCOLLN(x)
#define SERIAL_ECHOPAIR(pre,value)     SERIAL_PROTOCOLPAIR(pre, value)
#define SERIAL_ECHOPGM(x)              SERIAL_PROTOCOLPGM(x)
#define SERIAL_ECHOLNPGM(x)            SERIAL_PROTOCOLLNPGM(x)

#define SERIAL_ERROR_START()           (serialprintPGM(errormagic))
#define SERIAL_ERRORLNPGM(x)           SERIAL_PROTOCOLLNPGM(x)
#define SERIAL_ERRORPGM(x)             SERIAL_PROTOCOLPGM(x)

void serial_echopair_P(const char* s_P, const char *v);
void serial_echopair_P(const char* s_P, char v);
void serial_echopair_P(const char* s_P, int32_t v);
void serial_echopair_P(const char* s_P, float32_t v);
void serial_echopair_P(const char* s_P, float64_t v);
void serial_echopair_P(const char* s_P, uint32_t v);
FORCE_INLINE void serial_echopair_P(const char* s_P, uint8_t v) { serial_echopair_P(s_P, static_cast<int32_t>(v)); }
FORCE_INLINE void serial_echopair_P(const char* s_P, uint16_t v) { serial_echopair_P(s_P, static_cast<int32_t>(v)); }
FORCE_INLINE void serial_echopair_P(const char* s_P, bool v) { serial_echopair_P(s_P, static_cast<int32_t>(v)); }
FORCE_INLINE void serial_echopair_P(const char* s_P, void *v) { serial_echopair_P(s_P, reinterpret_cast<uint32_t>(v)); }

FORCE_INLINE void serialprintPGM(const char* str) {
  while (char ch = *(str++)) MarlinSerial::write(ch);
}

#endif /* SERIAL_H_ */
