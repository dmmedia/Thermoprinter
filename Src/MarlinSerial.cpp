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

    hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = baud;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}
	__HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_RXNE);

	uint16_t baud_setting;

  #if TX_BUFFER_SIZE > 0
    __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_TXE);
    __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_TC);
    _written = false;
  #endif
}

void MarlinSerial::end() {
	HAL_UART_DeInit(&hlpuart1);
}

// Preinstantiate
MarlinSerial customizedSerial;

#endif // !USBCON && (UBRRH || UBRR0H || UBRR1H || UBRR2H || UBRR3H)

// For USB targets use the UART for BT interfacing
#if ENABLED(BLUETOOTH)
  MarlinSerial bluetoothSerial;
#endif
