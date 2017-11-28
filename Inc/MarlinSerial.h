/*
 * HardwareSerial.h
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#ifndef MARLINSERIAL_H_
#define MARLINSERIAL_H_

#include <stdint.h>

#ifndef SERIAL_PORT
  #define SERIAL_PORT 0
#endif

// The presence of the UBRRH register is used to detect a UART.
#define UART_PRESENT(port) ((port == 0 && defined(LPUART1)) || \
                            (port == 1 && defined(USART1)) || (port == 2 && defined(USART2)))

extern UART_HandleTypeDef hlpuart1;

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

__attribute__((always_inline)) inline void store_char(unsigned char c);

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

/** @brief  Disable the specified UART interrupt.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __INTERRUPT__: specifies the UART interrupt source to disable.
  *          This parameter can be one of the following values:
  *            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
  *            @arg @ref UART_IT_CM   Character match interrupt
  *            @arg @ref UART_IT_CTS  CTS change interrupt
  *            @arg @ref UART_IT_LBD  LIN Break detection interrupt
  *            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
  *            @arg @ref UART_IT_TC   Transmission complete interrupt
  *            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
  *            @arg @ref UART_IT_IDLE Idle line detection interrupt
  *            @arg @ref UART_IT_PE   Parity Error interrupt
  *            @arg @ref UART_IT_ERR  Error interrupt (Frame error, noise error, overrun error)
  * @retval None
  */
void UartDisableIT(UART_HandleTypeDef *huart, uint32_t source);

/** @brief  Enable the specified UART interrupt.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __INTERRUPT__: specifies the UART interrupt source to enable.
  *          This parameter can be one of the following values:
  *            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
  *            @arg @ref UART_IT_CM   Character match interrupt
  *            @arg @ref UART_IT_CTS  CTS change interrupt
  *            @arg @ref UART_IT_LBD  LIN Break detection interrupt
  *            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
  *            @arg @ref UART_IT_TC   Transmission complete interrupt
  *            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
  *            @arg @ref UART_IT_IDLE Idle line detection interrupt
  *            @arg @ref UART_IT_PE   Parity Error interrupt
  *            @arg @ref UART_IT_ERR  Error interrupt (Frame error, noise error, overrun error)
  * @retval None
  */
void UartEnableIT(UART_HandleTypeDef *huart, uint32_t source);

/** @brief  Check whether the specified UART interrupt source is enabled or not.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __IT__: specifies the UART interrupt source to check.
  *          This parameter can be one of the following values:
  *            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
  *            @arg @ref UART_IT_CM   Character match interrupt
  *            @arg @ref UART_IT_CTS  CTS change interrupt
  *            @arg @ref UART_IT_LBD  LIN Break detection interrupt
  *            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
  *            @arg @ref UART_IT_TC   Transmission complete interrupt
  *            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
  *            @arg @ref UART_IT_IDLE Idle line detection interrupt
  *            @arg @ref UART_IT_ERR  Error interrupt (Frame error, noise error, overrun error)
  *            @arg @ref UART_IT_PE   Parity Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
ITStatus UartGetITSource(UART_HandleTypeDef *huart, uint32_t source);

/** @brief  Check whether the specified UART interrupt has occurred or not.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __IT__: specifies the UART interrupt to check.
  *          This parameter can be one of the following values:
  *            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
  *            @arg @ref UART_IT_CM   Character match interrupt
  *            @arg @ref UART_IT_CTS  CTS change interrupt
  *            @arg @ref UART_IT_LBD  LIN Break detection interrupt
  *            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
  *            @arg @ref UART_IT_TC   Transmission complete interrupt
  *            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
  *            @arg @ref UART_IT_IDLE Idle line detection interrupt
  *            @arg @ref UART_IT_ORE  Overrun Error interrupt
  *            @arg @ref UART_IT_NE   Noise Error interrupt
  *            @arg @ref UART_IT_FE   Framing Error interrupt
  *            @arg @ref UART_IT_PE   Parity Error interrupt
  * @retval The new state of __IT__ (TRUE or FALSE).
  */
ITStatus UartGetIT(UART_HandleTypeDef *huart, uint32_t source);

/** @brief  Set a specific UART request flag.
  * @param  __HANDLE__: specifies the UART Handle.
  * @param  __REQ__: specifies the request flag to set
  *          This parameter can be one of the following values:
  *            @arg @ref UART_AUTOBAUD_REQUEST Auto-Baud Rate Request
  *            @arg @ref UART_SENDBREAK_REQUEST Send Break Request
  *            @arg @ref UART_MUTE_MODE_REQUEST Mute Mode Request
  *            @arg @ref UART_RXDATA_FLUSH_REQUEST Receive Data flush Request
  *            @arg @ref UART_TXDATA_FLUSH_REQUEST Transmit data flush Request
  * @retval None
  */
void UartSendReq(UART_HandleTypeDef *huart, uint32_t req);

#endif // !USBCON

#endif /* MARLINSERIAL_H_ */
