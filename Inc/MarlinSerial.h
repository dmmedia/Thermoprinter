#pragma once

/*
 * MarlinSerial.h
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#include <stdint.h>
#include <stddef.h>
#include <stm32l0xx.h>
#include "typedefs.h"

namespace MarlinSerial {
	#ifndef SERIAL_PORT
	  #define SERIAL_PORT 0
	#endif

	#define UART_HWCONTROL_NONE                  ((uint32_t)0x00000000U)                          /*!< No hardware control       */
	#define UART_HWCONTROL_RTS                   ((uint32_t)USART_CR3_RTSE)                       /*!< Request To Send           */
	#define UART_HWCONTROL_CTS                   ((uint32_t)USART_CR3_CTSE)                       /*!< Clear To Send             */
	#define UART_HWCONTROL_RTS_CTS               ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE))    /*!< Request and Clear To Send */

	/**       Elements values convention: 0000ZZZZ00000000b
	  *           - ZZZZ  : Flag position in the ISR register(4bits)
	  */
	#define UART_IT_ORE                         ((uint32_t)0x0300)                  /*!< UART overrun error interruption */
	#define UART_IT_NE                          ((uint32_t)0x0200)                  /*!< UART noise error interruption   */
	#define UART_IT_FE                          ((uint32_t)0x0100)                  /*!< UART frame error interruption   */

	#define UART_AUTOBAUD_REQUEST               ((uint32_t)USART_RQR_ABRRQ)        /*!< Auto-Baud Rate Request      */
	#define UART_SENDBREAK_REQUEST              ((uint32_t)USART_RQR_SBKRQ)        /*!< Send Break Request          */
	#define UART_MUTE_MODE_REQUEST              ((uint32_t)USART_RQR_MMRQ)         /*!< Mute Mode Request           */
	#define UART_RXDATA_FLUSH_REQUEST           ((uint32_t)USART_RQR_RXFRQ)        /*!< Receive Data flush Request  */
	#define UART_TXDATA_FLUSH_REQUEST           ((uint32_t)USART_RQR_TXFRQ)        /*!< Transmit data flush Request */

	#define UART_IT_MASK                        ((uint32_t)0x001FU)                 /*!< UART interruptions flags mask */

	/**
	  * @brief UART Init Structure definition
	  */
	typedef struct
	{
	  uint32_t BaudRate;                  /*!< This member configures the UART communication baud rate.
											   The baud rate register is computed using the following formula:
											   - If oversampling is 16 or in LIN mode,
												  Baud Rate Register = ((PCLKx) / ((huart->Init.BaudRate)))
											   - If oversampling is 8,
												  Baud Rate Register[15:4] = ((2 * PCLKx) / ((huart->Init.BaudRate)))[15:4]
												  Baud Rate Register[3] =  0
												  Baud Rate Register[2:0] =  (((2 * PCLKx) / ((huart->Init.BaudRate)))[3:0]) >> 1      */

	  uint32_t WordLength;                /*!< Specifies the number of data bits transmitted or received in a frame.
											   This parameter can be a value of @ref UARTEx_Word_Length. */

	  uint32_t StopBits;                  /*!< Specifies the number of stop bits transmitted.
											   This parameter can be a value of @ref UART_Stop_Bits. */

	  uint32_t Parity;                    /*!< Specifies the parity mode.
											   This parameter can be a value of @ref UART_Parity
											   @note When parity is enabled, the computed parity is inserted
													 at the MSB position of the transmitted data (9th bit when
													 the word length is set to 9 data bits; 8th bit when the
													 word length is set to 8 data bits). */

	  uint32_t Mode;                      /*!< Specifies whether the Receive or Transmit mode is enabled or disabled.
											   This parameter can be a value of @ref UART_Mode. */

	  uint32_t HwFlowCtl;                 /*!< Specifies whether the hardware flow control mode is enabled
											   or disabled.
											   This parameter can be a value of @ref UART_Hardware_Flow_Control. */

	  uint32_t OverSampling;              /*!< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to f_PCLK/8).
											   This parameter can be a value of @ref UART_Over_Sampling. */

	  uint32_t OneBitSampling;            /*!< Specifies whether a single sample or three samples' majority vote is selected.
											   Selecting the single sample method increases the receiver tolerance to clock
											   deviations. This parameter can be a value of @ref UART_OneBit_Sampling. */
	}UartInitTypeDef;

	/**
	  * @brief  UART Advanced Features initalization structure definition
	  */
	typedef struct
	{
	  uint32_t AdvFeatureInit;        /*!< Specifies which advanced UART features is initialized. Several
										   Advanced Features may be initialized at the same time .
										   This parameter can be a value of @ref UART_Advanced_Features_Initialization_Type. */

	  uint32_t TxPinLevelInvert;      /*!< Specifies whether the TX pin active level is inverted.
										   This parameter can be a value of @ref UART_Tx_Inv.  */

	  uint32_t RxPinLevelInvert;      /*!< Specifies whether the RX pin active level is inverted.
										   This parameter can be a value of @ref UART_Rx_Inv.  */

	  uint32_t DataInvert;            /*!< Specifies whether data are inverted (positive/direct logic
										   vs negative/inverted logic).
										   This parameter can be a value of @ref UART_Data_Inv. */

	  uint32_t Swap;                  /*!< Specifies whether TX and RX pins are swapped.
										   This parameter can be a value of @ref UART_Rx_Tx_Swap. */

	  uint32_t OverrunDisable;        /*!< Specifies whether the reception overrun detection is disabled.
										   This parameter can be a value of @ref UART_Overrun_Disable. */

	  uint32_t DMADisableonRxError;   /*!< Specifies whether the DMA is disabled in case of reception error.
										   This parameter can be a value of @ref UART_DMA_Disable_on_Rx_Error. */

	  uint32_t AutoBaudRateEnable;    /*!< Specifies whether auto Baud rate detection is enabled.
										   This parameter can be a value of @ref UART_AutoBaudRate_Enable */

	  uint32_t AutoBaudRateMode;      /*!< If auto Baud rate detection is enabled, specifies how the rate
										   detection is carried out.
										   This parameter can be a value of @ref UART_AutoBaud_Rate_Mode. */

	  uint32_t MSBFirst;              /*!< Specifies whether MSB is sent first on UART line.
										   This parameter can be a value of @ref UART_MSB_First. */
	} UartAdvFeatureInitTypeDef;

	/**
	  * @brief HAL UART State structures definition
	  * @note  HAL UART State value is a combination of 2 different substates: gState and RxState.
	  *        - gState contains UART state information related to global Handle management
	  *          and also information related to Tx operations.
	  *          gState value coding follow below described bitmap :
	  *          b7-b6  Error information
	  *             00 : No Error
	  *             01 : (Not Used)
	  *             10 : Timeout
	  *             11 : Error
	  *          b5     IP initilisation status
	  *             0  : Reset (IP not initialized)
	  *             1  : Init done (IP not initialized. HAL UART Init function already called)
	  *          b4-b3  (not used)
	  *             xx : Should be set to 00
	  *          b2     Intrinsic process state
	  *             0  : Ready
	  *             1  : Busy (IP busy with some configuration or internal operations)
	  *          b1     (not used)
	  *             x  : Should be set to 0
	  *          b0     Tx state
	  *             0  : Ready (no Tx operation ongoing)
	  *             1  : Busy (Tx operation ongoing)
	  *        - RxState contains information related to Rx operations.
	  *          RxState value coding follow below described bitmap :
	  *          b7-b6  (not used)
	  *             xx : Should be set to 00
	  *          b5     IP initilisation status
	  *             0  : Reset (IP not initialized)
	  *             1  : Init done (IP not initialized)
	  *          b4-b2  (not used)
	  *            xxx : Should be set to 000
	  *          b1     Rx state
	  *             0  : Ready (no Rx operation ongoing)
	  *             1  : Busy (Rx operation ongoing)
	  *          b0     (not used)
	  *             x  : Should be set to 0.
	  */
	typedef enum
	{
	  UART_STATE_RESET             = 0x00U,   /*!< Peripheral is not initialized
													   Value is allowed for gState and RxState */
	  UART_STATE_READY             = 0x20U,   /*!< Peripheral Initialized and ready for use
													   Value is allowed for gState and RxState */
	  UART_STATE_BUSY              = 0x24U,   /*!< an internal process is ongoing
													   Value is allowed for gState only */
	  UART_STATE_BUSY_TX           = 0x21U,   /*!< Data Transmission process is ongoing
													   Value is allowed for gState only */
	  UART_STATE_BUSY_RX           = 0x22U,   /*!< Data Reception process is ongoing
													   Value is allowed for RxState only */
	  UART_STATE_BUSY_TX_RX        = 0x23U,   /*!< Data Transmission and Reception process is ongoing
													   Not to be used for neither gState nor RxState.
													   Value is result of combination (Or) between gState and RxState values */
	  UART_STATE_TIMEOUT           = 0xA0U,   /*!< Timeout state
													   Value is allowed for gState only */
	  UART_STATE_ERROR             = 0xE0U    /*!< Error
													   Value is allowed for gState only */
	}UartStateTypeDef;

	/**
	  * @brief  HAL UART Error Code structure definition
	  */
	typedef enum
	{
	  UART_ERROR_NONE      = 0x00,    /*!< No error            */
	  UART_ERROR_PE        = 0x01,    /*!< Parity error        */
	  UART_ERROR_NE        = 0x02,    /*!< Noise error         */
	  UART_ERROR_FE        = 0x04,    /*!< frame error         */
	  UART_ERROR_ORE       = 0x08,    /*!< Overrun error       */
	  UART_ERROR_DMA       = 0x10,    /*!< DMA transfer error  */
	  UART_ERROR_BUSY      = 0x20     /*!< Busy Error          */
	}UartErrorTypeDef;

	/**
	  * @brief  UART handle Structure definition
	  */
	typedef struct
	{
	  USART_TypeDef            *Instance;        /*!< UART registers base address        */

	  UartInitTypeDef         Init;             /*!< UART communication parameters      */

	  UartAdvFeatureInitTypeDef AdvancedInit;   /*!< UART Advanced Features initialization parameters */

	  uint8_t                  *pTxBuffPtr;      /*!< Pointer to UART Tx transfer Buffer */

	  uint16_t                 TxXferSize;       /*!< UART Tx Transfer size              */

	  __IO uint16_t            TxXferCount;      /*!< UART Tx Transfer Counter           */

	  uint8_t                  *pRxBuffPtr;      /*!< Pointer to UART Rx transfer Buffer */

	  uint16_t                 RxXferSize;       /*!< UART Rx Transfer size              */

	  __IO uint16_t            RxXferCount;      /*!< UART Rx Transfer Counter           */

	  uint16_t                 Mask;             /*!< UART Rx RDR register mask          */

	  HandleLockTypeDef           Lock;            /*!< Locking object                     */

	  __IO UartStateTypeDef    gState;      /*!< UART state information related to global Handle management
													  and also related to Tx operations.
													  This parameter can be a value of @ref HAL_UART_StateTypeDef */

	  __IO UartStateTypeDef    RxState;     /*!< UART state information related to Rx operations.
													  This parameter can be a value of @ref HAL_UART_StateTypeDef */

	  __IO UartErrorTypeDef             ErrorCode;       /*!< UART Error code                    */

	} UartHandle;

	/** @brief  Disable UART.
	  * @param  __HANDLE__: specifies the UART Handle.
	  * @retval None
	  */
	FORCE_INLINE void UART_Disable(UartHandle *huart) {
		huart->Instance->CR1 &= ~(static_cast<uint32_t>(USART_CR1_UE));
	}

	/** @brief  Enable UART.
	  * @param  __HANDLE__: specifies the UART Handle.
	  * @retval None
	  */
	FORCE_INLINE void UART_Enable(UartHandle * huart) {
		huart->Instance->CR1 |= static_cast<uint32_t>(USART_CR1_UE);
	}

	constexpr uint32_t UART_ADVFEATURE_NO_INIT 					= 0x00000000U;         /*!< No advanced feature initialization       */
	constexpr uint32_t UART_ADVFEATURE_TXINVERT_INIT 			= 0x00000001U;         /*!< TX pin active level inversion            */
	constexpr uint32_t UART_ADVFEATURE_RXINVERT_INIT 			= 0x00000002U;         /*!< RX pin active level inversion            */
	constexpr uint32_t UART_ADVFEATURE_DATAINVERT_INIT 			= 0x00000004U;         /*!< Binary data inversion                    */
	constexpr uint32_t UART_ADVFEATURE_SWAP_INIT 				= 0x00000008U;         /*!< TX/RX pins swap                          */
	constexpr uint32_t UART_ADVFEATURE_RXOVERRUNDISABLE_INIT 	= 0x00000010U;         /*!< RX overrun disable                       */
	constexpr uint32_t UART_ADVFEATURE_DMADISABLEONERROR_INIT	= 0x00000020U;         /*!< DMA disable on Reception Error           */
	constexpr uint32_t UART_ADVFEATURE_AUTOBAUDRATE_INIT 		= 0x00000040U;         /*!< Auto Baud rate detection initialization  */
	constexpr uint32_t UART_ADVFEATURE_MSBFIRST_INIT 			= 0x00000080U;         /*!< Most significant bit sent/received first */

	constexpr uint32_t UART_ADVFEATURE_AUTOBAUDRATE_DISABLE 	= 0x00000000U;          /*!< RX Auto Baud rate detection enable  */
	constexpr uint32_t UART_ADVFEATURE_AUTOBAUDRATE_ENABLE 		= USART_CR2_ABREN;      /*!< RX Auto Baud rate detection disable */

	#define UART_CR1_FIELDS  ((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | \
										 USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8)) /*!< UART or USART CR1 fields of parameters set by UART_SetConfig API */

	#define UART_LPUART_BRR_MIN           ((uint32_t)0x00000300)  /* LPUART BRR minimum authorized value */
	#define UART_LPUART_BRR_MAX           ((uint32_t)0x000FFFFF)  /* LPUART BRR maximum authorized value */

	constexpr uint32_t UART_OVERSAMPLING_16 = 0x00000000U;         /*!< Oversampling by 16 */
	constexpr uint32_t UART_OVERSAMPLING_8  = USART_CR1_OVER8;     /*!< Oversampling by 8  */

	#define HAL_UART_TIMEOUT_VALUE              0x1FFFFFF                           /*!< UART polling-based communications time-out value */

	#define UART_WORDLENGTH_7B                  ((uint32_t)USART_CR1_M1)   /*!< 7-bit long UART frame */
	#define UART_WORDLENGTH_8B                  ((uint32_t)0x00000000U)    /*!< 8-bit long UART frame */
	#define UART_WORDLENGTH_9B                  ((uint32_t)USART_CR1_M0)   /*!< 9-bit long UART frame */

	#define UART_STOPBITS_1                     ((uint32_t)0x00000000U)               /*!< UART frame with 1 stop bit    */
	#define UART_STOPBITS_1_5                   (USART_CR2_STOP_0 | USART_CR2_STOP_1) /*!< UART frame with 1.5 stop bits */
	#define UART_STOPBITS_2                      USART_CR2_STOP_1                     /*!< UART frame with 2 stop bits   */

	#define UART_PARITY_NONE                    ((uint32_t)0x00000000U)                        /*!< No parity   */
	#define UART_PARITY_EVEN                    ((uint32_t)USART_CR1_PCE)                      /*!< Even parity */
	#define UART_PARITY_ODD                     ((uint32_t)(USART_CR1_PCE | USART_CR1_PS))     /*!< Odd parity  */

	#define UART_MODE_RX                        ((uint32_t)USART_CR1_RE)                    /*!< RX mode        */
	#define UART_MODE_TX                        ((uint32_t)USART_CR1_TE)                    /*!< TX mode        */
	#define UART_MODE_TX_RX                     ((uint32_t)(USART_CR1_TE |USART_CR1_RE))    /*!< RX and TX mode */

	#define UART_ONE_BIT_SAMPLE_DISABLE         ((uint32_t)0x00000000U)         /*!< One-bit sampling disable */
	#define UART_ONE_BIT_SAMPLE_ENABLE          ((uint32_t)USART_CR3_ONEBIT)    /*!< One-bit sampling enable  */

	/** @defgroup UART_Interrupt_definition   UART Interrupts Definition
	  *        Elements values convention: 000ZZZZZ0XXYYYYYb
	  *           - YYYYY  : Interrupt source position in the XX register (5bits)
	  *           - XX  : Interrupt source register (2bits)
	  *                 - 01: CR1 register
	  *                 - 10: CR2 register
	  *                 - 11: CR3 register
	  *           - ZZZZZ  : Flag position in the ISR register(5bits)
	  * @{
	  */
	#define UART_IT_PE                          ((uint32_t)0x0028)                  /*!< UART parity error interruption                 */
	#define UART_IT_TXE                         ((uint32_t)0x0727)                  /*!< UART transmit data register empty interruption */
	#define UART_IT_TC                          ((uint32_t)0x0626)                  /*!< UART transmission complete interruption        */
	#define UART_IT_RXNE                        ((uint32_t)0x0525)                  /*!< UART read data register not empty interruption */
	#define UART_IT_IDLE                        ((uint32_t)0x0424)                  /*!< UART idle interruption                         */
	#define UART_IT_LBD                         ((uint32_t)0x0846)                  /*!< UART LIN break detection interruption          */
	#define UART_IT_CTS                         ((uint32_t)0x096A)                  /*!< UART CTS interruption                          */
	#define UART_IT_CM                          ((uint32_t)0x112E)                  /*!< UART character match interruption              */
	#define UART_IT_WUF                         ((uint32_t)0x1476)                  /*!< UART wake-up from stop mode interruption       */

	/** @brief  Check whether the specified UART flag is set or not.
	  * @param  __HANDLE__: specifies the UART Handle.
	  * @param  __FLAG__: specifies the flag to check.
	  *        This parameter can be one of the following values:
	  *            @arg @ref UART_FLAG_REACK Receive enable acknowledge flag
	  *            @arg @ref UART_FLAG_TEACK Transmit enable acknowledge flag
	  *            @arg @ref UART_FLAG_WUF   Wake up from stop mode flag
	  *            @arg @ref UART_FLAG_RWU   Receiver wake up flag (if the UART in mute mode)
	  *            @arg @ref UART_FLAG_SBKF  Send Break flag
	  *            @arg @ref UART_FLAG_CMF   Character match flag
	  *            @arg @ref UART_FLAG_BUSY  Busy flag
	  *            @arg @ref UART_FLAG_ABRF  Auto Baud rate detection flag
	  *            @arg @ref UART_FLAG_ABRE  Auto Baud rate detection error flag
	  *            @arg @ref UART_FLAG_EOBF  End of block flag
	  *            @arg @ref UART_FLAG_RTOF  Receiver timeout flag
	  *            @arg @ref UART_FLAG_CTS   CTS Change flag
	  *            @arg @ref UART_FLAG_LBDF  LIN Break detection flag
	  *            @arg @ref UART_FLAG_TXE   Transmit data register empty flag
	  *            @arg @ref UART_FLAG_TC    Transmission Complete flag
	  *            @arg @ref UART_FLAG_RXNE  Receive data register not empty flag
	  *            @arg @ref UART_FLAG_IDLE  Idle Line detection flag
	  *            @arg @ref UART_FLAG_ORE   Overrun Error flag
	  *            @arg @ref UART_FLAG_NE    Noise Error flag
	  *            @arg @ref UART_FLAG_FE    Framing Error flag
	  *            @arg @ref UART_FLAG_PE    Parity Error flag
	  * @retval The new state of __FLAG__ (TRUE or FALSE).
	  */
	#define UART_GET_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->Instance->ISR & (__FLAG__)) == (__FLAG__))

	#define UART_MAX_DELAY      0xFFFFFFFFU

	#define LOCK_HANDLE(__HANDLE__)                                               \
								  do{                                            \
									  if((__HANDLE__)->Lock == HAL_LOCKED)       \
									  {                                          \
										 return HAL_BUSY;                        \
									  }                                          \
									  else                                       \
									  {                                          \
										 (__HANDLE__)->Lock = HAL_LOCKED;        \
									  }                                          \
									}while (0)

	FORCE_INLINE void UART_UnlockHandle(UartHandle *huart) {
		huart->Lock = HANDLE_UNLOCKED;
	}

	/** @brief  BRR division operation to set BRR register in 8-bit oversampling mode.
	  * @param  __PCLK__: UART clock.
	  * @param  __BAUD__: Baud rate set by the user.
	  * @retval Division result
	  */
	#define UART_DIV_SAMPLING8(__PCLK__, __BAUD__)   ((((__PCLK__)*2U) + ((__BAUD__)/2U)) / (__BAUD__))

	/** @brief  BRR division operation to set BRR register in 16-bit oversampling mode.
	  * @param  __PCLK__: UART clock.
	  * @param  __BAUD__: Baud rate set by the user.
	  * @retval Division result
	  */
	FORCE_INLINE uint32_t UART_DivSampling16(uint32_t pclk, uint32_t baud) {
		return ((pclk + (baud / 2U)) / baud) & 0xFFFFU;
	}

	/** @brief  Check whether or not UART instance is Low Power UART.
	  * @param  __HANDLE__: specifies the UART Handle.
	  * @retval SET (instance is LPUART) or RESET (instance isn't LPUART)
	  */
	#define UART_INSTANCE_LOWPOWER(__HANDLE__) (((__HANDLE__)->Instance == LPUART1) ? SET : RESET )

	/** @brief  BRR division operation to set BRR register with LPUART.
	  * @param  __PCLK__: LPUART clock.
	  * @param  __BAUD__: Baud rate set by the user.
	  * @retval Division result
	  */
	FORCE_INLINE uint32_t UART_DivLpUart(uint32_t pclk, uint32_t baud) {
		return (((static_cast<uint64_t>(pclk) * 256U) + (baud / 2U)) / baud);
	}

	extern UartHandle hlpuart1;
	extern UartHandle huart2;

	extern uint8_t rx_buf[RXBUF_LEN];
	extern uint8_t tx_buf[TXBUF_LEN];

	// xx_i - counter of input bytes (tx - pushed for transmit, rx - received)
	// xx_o - counter of output bytes (tx - transmitted, rx - parsed)
	// xx_e - counter of echoed bytes
	extern volatile uint16_t rx_i;
	extern volatile uint16_t tx_o;
	extern uint16_t rx_o;
	extern uint16_t rx_e;
	extern uint16_t tx_i;

	#define DEC 10
	#define HEX 16
	#define OCT 8
	#define BIN 2
	#define BYTE 0

	// Define constants and variables for buffering incoming serial data.  We're
	// using a ring buffer (I think), in which rx_buffer_head is the index of the
	// location to which to write the next incoming character and rx_buffer_tail
	// is the index of the location from which to read.
	// 256 is the max limit due to uint8_t head and tail. Use only powers of 2. (...,16,32,64,128,256)
	#ifndef RX_BUFFER_SIZE
		#define RX_BUFFER_SIZE 128U
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

    extern ring_buffer_r rx_buffer;
    #if TX_BUFFER_SIZE > 0
    	extern ring_buffer_t tx_buffer;
		extern bool written;
    #endif

	FORCE_INLINE void store_char(uint8_t const c) {
		noInterrupts();
		const uint8_t h = rx_buffer.head;
		const uint8_t i = (h + 1U) & static_cast<uint8_t>(RX_BUFFER_SIZE - 1);

		// if we should be storing the received character into the location
		// just before the tail (meaning that the head would advance to the
		// current location of the tail), we're about to overflow the buffer
		// and so we don't write the character or advance the head.
		if (i != rx_buffer.tail) {
			rx_buffer.buffer[h] = c;
			rx_buffer.head = i;
		}
		interrupts();
	}

    void printNumber(uint32_t, const uint8_t);
    void begin(const uint32_t);
    void end();
    void flush(void);
    uint8_t available(void);
    int32_t read(void);
    void write(const uint8_t c);
#if TX_BUFFER_SIZE > 0
    uint8_t availableForWrite(void);
    void flushTX(void);
#endif
    void writeNoHandshake(const uint8_t c);
    FORCE_INLINE void write(const char* str) { while (*str) write(*str++); }
    FORCE_INLINE void write(const uint8_t* buffer, size_t size) { while (size--) write(*buffer++); }
    FORCE_INLINE void print(const char* str) { write(str); }

    void print(char const, int32_t const = BYTE);
    void print(uint8_t, int32_t = BYTE);
    void print(int32_t const , int32_t const = DEC);
    void print(uint32_t const, int32_t const = DEC);
    void print(double, int = 2);

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
	void UartDisableIT(const UartHandle * const huart, const uint32_t source);

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
	void UartEnableIT(const UartHandle * const huart, uint32_t const source);

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
	ITStatus UartGetITSource(const UartHandle * const huart, uint32_t const source);

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
	ITStatus UartGetIT(const UartHandle * const huart, uint32_t const source);

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
	void UartSendReq(const UartHandle * const huart, uint32_t const req);

	void UartMspInit(const UartHandle* const huart);
	void UART_AdvFeatureConfig(const UartHandle * const huart);
	StatusTypeDef UartSetConfig(UartHandle *huart);

	/**
	  * @brief UART clock sources definition
	  */
	typedef enum
	{
	  UART_CLOCKSOURCE_PCLK1      = 0x00,    /*!< PCLK1 clock source     */
	  UART_CLOCKSOURCE_PCLK2      = 0x01,    /*!< PCLK2 clock source     */
	  UART_CLOCKSOURCE_HSI        = 0x02,    /*!< HSI clock source       */
	  UART_CLOCKSOURCE_SYSCLK     = 0x04,    /*!< SYSCLK clock source    */
	  UART_CLOCKSOURCE_LSE        = 0x08,    /*!< LSE clock source       */
	  UART_CLOCKSOURCE_UNDEFINED  = 0x10     /*!< Undefined clock source */
	}UART_ClockSourceTypeDef;

	FORCE_INLINE UART_ClockSourceTypeDef UART_GetClockSource(UartHandle *huart) {
		UART_ClockSourceTypeDef res = UART_CLOCKSOURCE_UNDEFINED;
	    if((huart)->Instance == USART1)
	    {
	    	switch(RCC_GET_USART1_SOURCE())
	    	{
	       		case RCC_USART1CLKSOURCE_PCLK2:
	       			res = UART_CLOCKSOURCE_PCLK2;
	       			break;
	       		case RCC_USART1CLKSOURCE_HSI:
	       			res = UART_CLOCKSOURCE_HSI;
	       			break;
	       		case RCC_USART1CLKSOURCE_SYSCLK:
	       			res = UART_CLOCKSOURCE_SYSCLK;
	       			break;
	       		case RCC_USART1CLKSOURCE_LSE:
	       			res = UART_CLOCKSOURCE_LSE;
	       			break;
	       		default:
	       			res = UART_CLOCKSOURCE_UNDEFINED;
	       			break;
	    	}
	    }
	    else if((huart)->Instance == USART2)
	    {
	    	switch(RCC_GET_USART2_SOURCE())
	    	{
	        	case RCC_USART2CLKSOURCE_PCLK1:
	        		res = UART_CLOCKSOURCE_PCLK1;
	        		break;
	        	case RCC_USART2CLKSOURCE_HSI:
	        		res = UART_CLOCKSOURCE_HSI;
	        		break;
	        	case RCC_USART2CLKSOURCE_SYSCLK:
	        		res = UART_CLOCKSOURCE_SYSCLK;
	        		break;
	        	case RCC_USART2CLKSOURCE_LSE:
	        		res = UART_CLOCKSOURCE_LSE;
	        		break;
	        	default:
	        		res = UART_CLOCKSOURCE_UNDEFINED;
	        		break;
	    	}
	    }
	    else if((huart)->Instance == LPUART1)
	    {
	    	switch(RCC_GET_LPUART1_SOURCE())
	    	{
	    		case RCC_LPUART1CLKSOURCE_PCLK1:
	    			res = UART_CLOCKSOURCE_PCLK1;
	    			break;
	    		case RCC_LPUART1CLKSOURCE_HSI:
	    			res = UART_CLOCKSOURCE_HSI;
	    			break;
	    		case RCC_LPUART1CLKSOURCE_SYSCLK:
	    			res = UART_CLOCKSOURCE_SYSCLK;
	    			break;
	    		case RCC_LPUART1CLKSOURCE_LSE:
	    			res = UART_CLOCKSOURCE_LSE;
	    			break;
	    		default:
	    			res = UART_CLOCKSOURCE_UNDEFINED;
	    			break;
	    	}
	    }
		else
		{
			res = UART_CLOCKSOURCE_UNDEFINED;
		}
	    return res;
	}

	//
	// @brief  Handle UART Communication Timeout.
	// @param  huart UART handle.
	// @param  Flag Specifies the UART flag to check
	// @param  Status Flag status (SET or RESET)
	// @param  Tickstart Tick start value
	// @param  Timeout Timeout duration
	// @retval Status
	//
	StatusTypeDef UART_WaitOnFlagUntilTimeout(UartHandle * const huart, uint32_t const Flag, FlagStatus const Status, uint32_t const Tickstart, uint32_t const Timeout);

	//
	// @brief Check the UART Idle State.
	// @param huart UART handle.
	// @retval HAL status
	///
	StatusTypeDef UART_CheckIdleState(UartHandle * const huart);

	//
	// @brief Initialize the UART mode according to the specified
	//        parameters in the UART_InitTypeDef and initialize the associated handle.
	// @param huart: UART handle.
	// @retval HAL status
	//
	StatusTypeDef UartInit(UartHandle * const huart);

	//
	// @brief DeInitialize the UART peripheral.
	// @param huart: UART handle.
	// @retval HAL status
	///
	StatusTypeDef UartDeInit(UartHandle * const huart);

	// LPUART1 init function
	void MX_LPUART1_UART_Init(const uint32_t baud);

	// USART2 init function
	void MX_USART2_UART_Init(const uint32_t baud);

	void UartMspDeInit(const UartHandle* const huart);

	void transmit(const UartHandle * const huart, const uint8_t byte);
}

extern "C" {
	void USART2_IRQHandler(void);
	void RNG_LPUART1_IRQHandler(void);
}
