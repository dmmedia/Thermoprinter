/*
 * serial.h
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#ifdef USBCON
  #include <HardwareSerial.h>
  #if ENABLED(BLUETOOTH)
    #define MYSERIAL bluetoothSerial
  #else
    #define MYSERIAL Serial
  #endif // BLUETOOTH
#else
  #include "MarlinSerial.h"
  #define MYSERIAL customizedSerial
#endif


#endif /* SERIAL_H_ */
