/*
 * HardwareSerial.h
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#ifndef MARLINSERIAL_H_
#define MARLINSERIAL_H_

#include "main.h"

#ifndef SERIAL_PORT
  #define SERIAL_PORT 0
#endif

// The presence of the UBRRH register is used to detect a UART.
#define UART_PRESENT(port) ((port == 0 && defined(LPUART1)) || \
                            (port == 1 && defined(USART1)) || (port == 2 && defined(USART2)))

#ifndef USBCON
  // Define constants and variables for buffering incoming serial data.  We're
  // using a ring buffer (I think), in which rx_buffer_head is the index of the
  // location to which to write the next incoming character and rx_buffer_tail
  // is the index of the location from which to read.
  // 256 is the max limit due to uint8_t head and tail. Use only powers of 2. (...,16,32,64,128,256)
  #ifndef RX_BUFFER_SIZE
    #define RX_BUFFER_SIZE 128
  #endif
  #ifndef TX_BUFFER_SIZE
    #define TX_BUFFER_SIZE 32
  #endif
  #if !((RX_BUFFER_SIZE == 256) ||(RX_BUFFER_SIZE == 128) ||(RX_BUFFER_SIZE == 64) ||(RX_BUFFER_SIZE == 32) ||(RX_BUFFER_SIZE == 16) ||(RX_BUFFER_SIZE == 8) ||(RX_BUFFER_SIZE == 4) ||(RX_BUFFER_SIZE == 2))
    #error "RX_BUFFER_SIZE has to be a power of 2 and >= 2"
  #endif
  #if !((TX_BUFFER_SIZE == 256) ||(TX_BUFFER_SIZE == 128) ||(TX_BUFFER_SIZE == 64) ||(TX_BUFFER_SIZE == 32) ||(TX_BUFFER_SIZE == 16) ||(TX_BUFFER_SIZE == 8) ||(TX_BUFFER_SIZE == 4) ||(TX_BUFFER_SIZE == 2) ||(TX_BUFFER_SIZE == 0))
    #error TX_BUFFER_SIZE has to be a power of 2 or 0
  #endif

  struct ring_buffer_r {
    unsigned char buffer[RX_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
  };

  #if TX_BUFFER_SIZE > 0
    struct ring_buffer_t {
      unsigned char buffer[TX_BUFFER_SIZE];
      volatile uint8_t head;
      volatile uint8_t tail;
    };
  #endif

  #if UART_PRESENT(SERIAL_PORT)
    extern ring_buffer_r rx_buffer;
    #if TX_BUFFER_SIZE > 0
      extern ring_buffer_t tx_buffer;
    #endif
  #endif

  FORCE_INLINE void store_char(unsigned char c) {
	CRITICAL_SECTION_START;
	  const uint8_t h = rx_buffer.head,
					i = (uint8_t)(h + 1) & (RX_BUFFER_SIZE - 1);

	  // if we should be storing the received character into the location
	  // just before the tail (meaning that the head would advance to the
	  // current location of the tail), we're about to overflow the buffer
	  // and so we don't write the character or advance the head.
	  if (i != rx_buffer.tail) {
		rx_buffer.buffer[h] = c;
		rx_buffer.head = i;
	  }
	CRITICAL_SECTION_END;
  }


class MarlinSerial {
public:
	MarlinSerial() {};
	virtual ~MarlinSerial() {};
    static void begin(const long);
    static void end();
    static void flush(void);
    static uint8_t available(void);
    static int read(void);
};


extern MarlinSerial customizedSerial;

#endif // !USBCON

// Use the UART for Bluetooth in AT90USB configurations
#if defined(USBCON) && ENABLED(BLUETOOTH)
  extern HardwareSerial bluetoothSerial;
#endif

#endif /* MARLINSERIAL_H_ */
