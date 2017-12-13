/*
 * HardwareSerial.cpp
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#include <stm32l0xx_hal.h>
#include <MarlinSerial.h>
#include "macros.h"
#include "Configuration.h"
#include "SREGEmulation.h"

#if !defined(USBCON) && (defined(LPUART1) || defined(USART1) || defined(USART2))

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

uint8_t rx_buf[RXBUF_LEN], tx_buf[TXBUF_LEN];
/* xx_i - counter of input bytes (tx - pushed for transmit, rx - received)
   xx_o - counter of output bytes (tx - transmitted, rx - parsed)
   xx_e - counter of echoed bytes */
volatile uint16_t rx_i = 0, tx_o = 0;
uint16_t rx_o = 0, rx_e = 0, tx_i = 0;

volatile uint8_t tx_busy = 0;

#if UART_PRESENT(SERIAL_PORT)
  ring_buffer_r rx_buffer = { { 0 }, 0, 0 };
  #if TX_BUFFER_SIZE > 0
    ring_buffer_t tx_buffer = { { 0 }, 0, 0 };
    static bool _written;
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

/* LPUART1 init function */
static void MX_LPUART1_UART_Init(const long baud)
{
	hlpuart1.Instance = LPUART1;
	hlpuart1.Init.BaudRate = baud;
	hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
	hlpuart1.Init.StopBits = UART_STOPBITS_1;
	hlpuart1.Init.Parity = UART_PARITY_NONE;
	hlpuart1.Init.Mode = UART_MODE_TX_RX;
	hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&hlpuart1) != HAL_OK)
	{
	  _Error_Handler(__FILE__, __LINE__);
	}
}

/* USART2 init function */
static void MX_USART2_UART_Init(const long baud)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = baud;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
	    _Error_Handler(__FILE__, __LINE__);
    }
}

void MarlinSerial::begin(const long baud) {

    MX_USART2_UART_Init(baud);
//	MX_LPUART1_UART_Init(baud);

	UartEnableIT(&huart2, UART_IT_RXNE);
//	UartEnableIT(&hlpuart1, UART_IT_RXNE);

  #if TX_BUFFER_SIZE > 0
	UartEnableIT(&huart2, UART_IT_TXE);
	UartEnableIT(&huart2, UART_IT_TC);
//	UartEnableIT(&hlpuart1, UART_IT_TXE);
//	UartEnableIT(&hlpuart1, UART_IT_TC);
    _written = false;
  #endif
}

void MarlinSerial::end() {
	HAL_UART_DeInit(&huart2);
//	HAL_UART_DeInit(&hlpuart1);
}

/**
* @brief This function handles AES, RNG and LPUART1 interrupts / LPUART1 wake-up interrupt through EXTI line 28.
*/
void AES_RNG_LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN AES_RNG_LPUART1_IRQn 0 */
    if((UartGetIT(&hlpuart1, UART_IT_RXNE) != RESET) &&
       (UartGetITSource(&hlpuart1, UART_IT_RXNE) != RESET))
    {
    	// read only of no errors, else skip byte
    	if (
    			(UartGetIT(&hlpuart1, UART_IT_NE) == RESET) &&
				(UartGetIT(&hlpuart1, UART_IT_FE) == RESET) &&
				(UartGetIT(&hlpuart1, UART_IT_PE) == RESET)
		) {
    		store_char((uint8_t)(hlpuart1.Instance->RDR & 0x00FF));
    	} else {
    		hlpuart1.Instance->RDR;
    	}
        /* Clear RXNE interrupt flag */
    	UartSendReq(&hlpuart1, UART_RXDATA_FLUSH_REQUEST);
    }
    if((UartGetIT(&hlpuart1, UART_IT_TXE) != RESET) &&
       (UartGetITSource(&hlpuart1, UART_IT_TXE) != RESET))
    {
        if (tx_buffer.head == tx_buffer.tail) {
        	UartDisableIT(&hlpuart1, UART_IT_TXE);
        	UartEnableIT(&hlpuart1, UART_IT_TC);
        } else {
        	// If interrupts are enabled, there must be more data in the output
        	// buffer. Send the next byte
        	const uint8_t t = tx_buffer.tail,
        	              c = tx_buffer.buffer[t];
        	tx_buffer.tail = (t + 1) & (TX_BUFFER_SIZE - 1);

            hlpuart1.Instance->TDR = c;
        }
    }
    if((UartGetIT(&hlpuart1, UART_IT_TC) != RESET) &&
       (UartGetITSource(&hlpuart1, UART_IT_TC) != RESET))
    {
        tx_busy = 0;
        UartDisableIT(&hlpuart1, UART_IT_TC);
    }

    /* And never call default handler */
    return;

    HAL_UART_IRQHandler(&hlpuart1);
}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
    if((UartGetIT(&huart2, UART_IT_RXNE) != RESET) &&
       (UartGetITSource(&huart2, UART_IT_RXNE) != RESET))
    {
    	// read only of no errors, else skip byte
    	if (
    			(UartGetIT(&huart2, UART_IT_NE) == RESET) &&
				(UartGetIT(&huart2, UART_IT_FE) == RESET) &&
				(UartGetIT(&huart2, UART_IT_PE) == RESET)
		) {
    		store_char((uint8_t)(huart2.Instance->RDR & 0x00FF));
    	} else {
    		huart2.Instance->RDR;
    	}
        /* Clear RXNE interrupt flag */
    	UartSendReq(&huart2, UART_RXDATA_FLUSH_REQUEST);
    }
    if((UartGetIT(&huart2, UART_IT_TXE) != RESET) &&
       (UartGetITSource(&huart2, UART_IT_TXE) != RESET))
    {
        if (tx_buffer.head == tx_buffer.tail) {
        	UartDisableIT(&huart2, UART_IT_TXE);
        	UartEnableIT(&huart2, UART_IT_TC);
        } else {
        	// If interrupts are enabled, there must be more data in the output
        	// buffer. Send the next byte
        	const uint8_t t = tx_buffer.tail,
        	              c = tx_buffer.buffer[t];
        	tx_buffer.tail = (t + 1) & (TX_BUFFER_SIZE - 1);

        	huart2.Instance->TDR = c;
        }
}
    if((UartGetIT(&huart2, UART_IT_TC) != RESET) &&
       (UartGetITSource(&huart2, UART_IT_TC) != RESET))
    {
        tx_busy = 0;
        UartDisableIT(&huart2, UART_IT_TC);
    }
    /* And never call default handler */
    return;

    HAL_UART_IRQHandler(&huart2);
}

void transmit(UART_HandleTypeDef *huart, uint8_t byte)
{
  tx_buf[TXBUF_MSK & tx_i] = byte;
  tx_i++;
  tx_busy = 1;
  UartEnableIT(huart, UART_IT_TXE);
}

void UartDisableIT(UART_HandleTypeDef *huart, uint32_t source) {
  (
	((((uint8_t)source) >> 5U) == 1U) ?
	  (huart->Instance->CR1 &= ~(1U << (source & UART_IT_MASK))) :
	  ((((uint8_t)source) >> 5U) == 2U) ?
		(huart->Instance->CR2 &= ~(1U << (source & UART_IT_MASK))) :
        (huart->Instance->CR3 &= ~(1U << (source & UART_IT_MASK)))
  );
}

void UartEnableIT(UART_HandleTypeDef *huart, uint32_t source) {
  (((((uint8_t)source) >> 5U) == 1U) ?
    (huart->Instance->CR1 |= (1U << (source & UART_IT_MASK))) :
    ((((uint8_t)source) >> 5U) == 2U) ?
	  (huart->Instance->CR2 |= (1U << (source & UART_IT_MASK))) :
      (huart->Instance->CR3 |= (1U << (source & UART_IT_MASK)))
  );
}

ITStatus UartGetITSource(UART_HandleTypeDef *huart, uint32_t source) {
  return ((((((uint8_t)source) >> 5U) == 1U) ?
	huart->Instance->CR1 :
	(((((uint8_t)source) >> 5U) == 2U) ?
	  huart->Instance->CR2 :
	    huart->Instance->CR3
	)
   ) & ((uint32_t)1U << (((uint16_t)source) & UART_IT_MASK))
  ) > 0 ? SET : RESET;
}

ITStatus UartGetIT(UART_HandleTypeDef *huart, uint32_t source) {
  return (huart->Instance->ISR & ((uint32_t)1U << (source >> 0x08U))) > 0 ? SET : RESET;
}

void UartSendReq(UART_HandleTypeDef *huart, uint32_t req) {
  (huart->Instance->RQR |= (uint32_t)req);
}

// Preinstantiate
MarlinSerial customizedSerial;

#endif // !USBCON && (UBRRH || UBRR0H || UBRR1H || UBRR2H || UBRR3H)

// For USB targets use the UART for BT interfacing
#if ENABLED(BLUETOOTH)
  MarlinSerial bluetoothSerial;
#endif
