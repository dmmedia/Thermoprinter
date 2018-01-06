/*
 * serial.h
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include "MarlinSerial.h"

#define MSG_T_MAXTEMP                       "MAXTEMP triggered"
#define MSG_T_MINTEMP                       "MINTEMP triggered"
#ifndef MSG_ERR_MAXTEMP
  #define MSG_ERR_MAXTEMP                     "Err: MAXTEMP"
#endif
#ifndef MSG_ERR_MINTEMP
  #define MSG_ERR_MINTEMP                     "Err: MINTEMP"
#endif

#define MSG_THERMOPRINTER "Thermoprinter"

#define SHORT_BUILD_VERSION "0.1"

#define MSG_OK                              "ok"

#define MSG_ERR_KILLED                      "Printer halted. kill() called!"

#define MSG_RESEND                          "Resend: "

#define MSG_ERR_LINE_NO                     "Line Number is not Last Line Number+1, Last Line: "

#define MSG_STOPPED_HEATER                  ", system stopped! Heater_ID: "

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
