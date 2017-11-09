/*
 * HardwareSerial.cpp
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#include <MarlinSerial.h>
#include "macros.h"

#if !defined(USBCON) && (defined(LPUART1) || defined(USART1) || defined(USART2))

  #if UART_PRESENT(SERIAL_PORT)
    ring_buffer_r rx_buffer = { { 0 }, 0, 0 };
    #if TX_BUFFER_SIZE > 0
      ring_buffer_t tx_buffer = { { 0 }, 0, 0 };
      static bool _written;
    #endif
  #endif

void MarlinSerial::flush(void) {
  // RX
  // don't reverse this or there may be problems if the RX interrupt
  // occurs after reading the value of rx_buffer_head but before writing
  // the value to rx_buffer_tail; the previous value of rx_buffer_head
  // may be written to rx_buffer_tail, making it appear as if the buffer
  // were full, not empty.
  CRITICAL_SECTION_START;
    rx_buffer.head = rx_buffer.tail;
  CRITICAL_SECTION_END;
}

uint8_t MarlinSerial::available(void) {
  CRITICAL_SECTION_START;
    const uint8_t h = rx_buffer.head,
                  t = rx_buffer.tail;
  CRITICAL_SECTION_END;
  return (uint8_t)(RX_BUFFER_SIZE + h - t) & (RX_BUFFER_SIZE - 1);
}

int MarlinSerial::read(void) {
  int v;
  CRITICAL_SECTION_START;
    const uint8_t t = rx_buffer.tail;
    if (rx_buffer.head == t)
      v = -1;
    else {
      v = rx_buffer.buffer[t];
      rx_buffer.tail = (uint8_t)(t + 1) & (RX_BUFFER_SIZE - 1);
    }
  CRITICAL_SECTION_END;
  return v;
}

void MarlinSerial::begin(const long baud) {
  uint16_t baud_setting;
  bool useU2X = true;

  #if F_CPU == 16000000UL && SERIAL_PORT == 0
    // hard-coded exception for compatibility with the bootloader shipped
    // with the Duemilanove and previous boards and the firmware on the 8U2
    // on the Uno and Mega 2560.
    if (baud == 57600) useU2X = false;
  #endif

  if (useU2X) {
    M_UCSRxA = _BV(M_U2Xx);
    baud_setting = (F_CPU / 4 / baud - 1) / 2;
  }
  else {
    M_UCSRxA = 0;
    baud_setting = (F_CPU / 8 / baud - 1) / 2;
  }

  // assign the baud_setting, a.k.a. ubbr (USART Baud Rate Register)
  M_UBRRxH = baud_setting >> 8;
  M_UBRRxL = baud_setting;

  SBI(M_UCSRxB, M_RXENx);
  SBI(M_UCSRxB, M_TXENx);
  SBI(M_UCSRxB, M_RXCIEx);
  #if TX_BUFFER_SIZE > 0
    CBI(M_UCSRxB, M_UDRIEx);
    _written = false;
  #endif
}

void MarlinSerial::end() {
  CBI(M_UCSRxB, M_RXENx);
  CBI(M_UCSRxB, M_TXENx);
  CBI(M_UCSRxB, M_RXCIEx);
  CBI(M_UCSRxB, M_UDRIEx);
}

// Preinstantiate
MarlinSerial customizedSerial;

#endif // !USBCON && (UBRRH || UBRR0H || UBRR1H || UBRR2H || UBRR3H)

// For USB targets use the UART for BT interfacing
#if ENABLED(BLUETOOTH)
  MarlinSerial bluetoothSerial;
#endif
