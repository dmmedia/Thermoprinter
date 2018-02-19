/*
 * HardwareSerial.cpp
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#include <stm32l0xx.h>
#include "typedefs.h"
#include "main.h"
#include "macros.h"
#include "gpio.h"
#include "Configuration.h"
#include "rcc.h"
#include <stdint.h>
#include <MarlinSerial.h>
#include "Timers.h"
#include "Thermoprinter.h"

namespace MarlinSerial {
	//
	// Private definitions and constants
	//

	//
	// @brief UART clock sources definition
	//
	typedef enum
	{
		UART_CLOCKSOURCE_PCLK1      = 0x00,    //!< PCLK1 clock source
		UART_CLOCKSOURCE_PCLK2      = 0x01,    //!< PCLK2 clock source
		UART_CLOCKSOURCE_HSI        = 0x02,    //!< HSI clock source
		UART_CLOCKSOURCE_SYSCLK     = 0x04,    //!< SYSCLK clock source
		UART_CLOCKSOURCE_LSE        = 0x08,    //!< LSE clock source
		UART_CLOCKSOURCE_UNDEFINED  = 0x10     //!< Undefined clock source
	} UART_ClockSourceTypeDef;

	//
	// @brief  HAL UART Error Code structure definition
	//
	typedef enum
	{
		UART_ERROR_NONE      = 0x00,    //!< No error
		UART_ERROR_PE        = 0x01,    //!< Parity error
		UART_ERROR_NE        = 0x02,    //!< Noise error
		UART_ERROR_FE        = 0x04,    //!< frame error
		UART_ERROR_ORE       = 0x08,    //!< Overrun error
		UART_ERROR_DMA       = 0x10,    //!< DMA transfer error
		UART_ERROR_BUSY      = 0x20     //!< Busy Error
	} UartErrorTypeDef;

	//
	// @brief HAL UART State structures definition
	// @note  HAL UART State value is a combination of 2 different substates: gState and RxState.
	//        - gState contains UART state information related to global Handle management
	//          and also information related to Tx operations.
	//          gState value coding follow below described bitmap :
	//          b7-b6  Error information
	//             00 : No Error
	//             01 : (Not Used)
	//             10 : Timeout
	//             11 : Error
	//          b5     IP initilisation status
	//             0  : Reset (IP not initialized)
	//             1  : Init done (IP not initialized. HAL UART Init function already called)
	//          b4-b3  (not used)
	//             xx : Should be set to 00
	//          b2     Intrinsic process state
	//             0  : Ready
	//             1  : Busy (IP busy with some configuration or internal operations)
	//          b1     (not used)
	//             x  : Should be set to 0
	//          b0     Tx state
	//             0  : Ready (no Tx operation ongoing)
	//             1  : Busy (Tx operation ongoing)
	//        - RxState contains information related to Rx operations.
	//          RxState value coding follow below described bitmap :
	//          b7-b6  (not used)
	//             xx : Should be set to 00
	//          b5     IP initilisation status
	//             0  : Reset (IP not initialized)
	//             1  : Init done (IP not initialized)
	//          b4-b2  (not used)
	//            xxx : Should be set to 000
	//          b1     Rx state
	//             0  : Ready (no Rx operation ongoing)
	//             1  : Busy (Rx operation ongoing)
	//          b0     (not used)
	//             x  : Should be set to 0.
	//
	typedef enum
	{
		UART_STATE_RESET             = 0x00U,   //!< Peripheral is not initialized
												//	   Value is allowed for gState and RxState
		UART_STATE_READY             = 0x20U,   //!< Peripheral Initialized and ready for use
												//	   Value is allowed for gState and RxState
		UART_STATE_BUSY              = 0x24U,   //!< an internal process is ongoing
												//	   Value is allowed for gState only
		UART_STATE_BUSY_TX           = 0x21U,   //!< Data Transmission process is ongoing
												//	   Value is allowed for gState only
		UART_STATE_BUSY_RX           = 0x22U,   //!< Data Reception process is ongoing
												//	   Value is allowed for RxState only
		UART_STATE_BUSY_TX_RX        = 0x23U,   //!< Data Transmission and Reception process is ongoing
												//	   Not to be used for neither gState nor RxState.
												//	   Value is result of combination (Or) between gState and RxState values
		UART_STATE_TIMEOUT           = 0xA0U,   //!< Timeout state
												//	   Value is allowed for gState only
		UART_STATE_ERROR             = 0xE0U    //!< Error
												//	   Value is allowed for gState only
	} UartStateTypeDef;

	//
	// @brief UART Init Structure definition
	//
	typedef struct
	{
		uint32_t BaudRate;                  //!< This member configures the UART communication baud rate.
											//   The baud rate register is computed using the following formula:
											//   - If oversampling is 16 or in LIN mode,
											//     Baud Rate Register = ((PCLKx) / ((huart->Init.BaudRate)))
											//   - If oversampling is 8,
											//     Baud Rate Register[15:4] = ((2 * PCLKx) / ((huart->Init.BaudRate)))[15:4]
											//     Baud Rate Register[3] =  0
											//     Baud Rate Register[2:0] =  (((2 * PCLKx) / ((huart->Init.BaudRate)))[3:0]) >> 1

		uint32_t WordLength;                //!< Specifies the number of data bits transmitted or received in a frame.
											//   This parameter can be a value of @ref UARTEx_Word_Length.

		uint32_t StopBits;                  //!< Specifies the number of stop bits transmitted.
											//   This parameter can be a value of @ref UART_Stop_Bits.

		uint32_t Parity;                    //!< Specifies the parity mode.
											//   This parameter can be a value of @ref UART_Parity
											//   @note When parity is enabled, the computed parity is inserted
											//         at the MSB position of the transmitted data (9th bit when
											//		   the word length is set to 9 data bits; 8th bit when the
											//		   word length is set to 8 data bits).

		uint32_t Mode;                      //!< Specifies whether the Receive or Transmit mode is enabled or disabled.
											//   This parameter can be a value of @ref UART_Mode.

		uint32_t HwFlowCtl;                 //!< Specifies whether the hardware flow control mode is enabled
											//   or disabled.
											//   This parameter can be a value of @ref UART_Hardware_Flow_Control.

		uint32_t OverSampling;              //!< Specifies whether the Over sampling 8 is enabled or disabled, to achieve higher speed (up to f_PCLK/8).
											//   This parameter can be a value of @ref UART_Over_Sampling.

		uint32_t OneBitSampling;            //!< Specifies whether a single sample or three samples' majority vote is selected.
											//   Selecting the single sample method increases the receiver tolerance to clock
											//   deviations. This parameter can be a value of @ref UART_OneBit_Sampling.
	} UartInitTypeDef;

	//
	// @brief  UART Advanced Features initalization structure definition
	//
	typedef struct
	{
		uint32_t AdvFeatureInit;        //!< Specifies which advanced UART features is initialized. Several
										//   Advanced Features may be initialized at the same time .
										//   This parameter can be a value of @ref UART_Advanced_Features_Initialization_Type.

		uint32_t TxPinLevelInvert;      //!< Specifies whether the TX pin active level is inverted.
										//   This parameter can be a value of @ref UART_Tx_Inv.

		uint32_t RxPinLevelInvert;      //!< Specifies whether the RX pin active level is inverted.
										//   This parameter can be a value of @ref UART_Rx_Inv.

		uint32_t DataInvert;            //!< Specifies whether data are inverted (positive/direct logic
										//   vs negative/inverted logic).
										//   This parameter can be a value of @ref UART_Data_Inv.

		uint32_t Swap;                  //!< Specifies whether TX and RX pins are swapped.
										//   This parameter can be a value of @ref UART_Rx_Tx_Swap.

		uint32_t OverrunDisable;        //!< Specifies whether the reception overrun detection is disabled.
										//   This parameter can be a value of @ref UART_Overrun_Disable.

		uint32_t DMADisableonRxError;   //!< Specifies whether the DMA is disabled in case of reception error.
										//   This parameter can be a value of @ref UART_DMA_Disable_on_Rx_Error.

		uint32_t AutoBaudRateEnable;    //!< Specifies whether auto Baud rate detection is enabled.
										//   This parameter can be a value of @ref UART_AutoBaudRate_Enable

		uint32_t AutoBaudRateMode;      //!< If auto Baud rate detection is enabled, specifies how the rate
										//   detection is carried out.
										//   This parameter can be a value of @ref UART_AutoBaud_Rate_Mode.

		uint32_t MSBFirst;              //!< Specifies whether MSB is sent first on UART line.
										//   This parameter can be a value of @ref UART_MSB_First.
	} UartAdvFeatureInitTypeDef;

	//
	// @brief  UART handle Structure definition
	//
	typedef struct
	{
		USART_TypeDef*				Instance;       //!< UART registers base address

		UartInitTypeDef         	Init;           //!< UART communication parameters

		UartAdvFeatureInitTypeDef	AdvancedInit;	//!< UART Advanced Features initialization parameters

		uint8_t*					pTxBuffPtr;     //!< Pointer to UART Tx transfer Buffer

		uint16_t                 	TxXferSize;     //!< UART Tx Transfer size

		__IO uint16_t            	TxXferCount;    //!< UART Tx Transfer Counter

		uint8_t*					pRxBuffPtr;     //!< Pointer to UART Rx transfer Buffer

		uint16_t                 	RxXferSize;     //!< UART Rx Transfer size

		__IO uint16_t            	RxXferCount;    //!< UART Rx Transfer Counter

		uint16_t                 	Mask;           //!< UART Rx RDR register mask

		HandleLockTypeDef           Lock;           //!< Locking object

		__IO UartStateTypeDef    	gState;      	//!< UART state information related to global Handle management
													//   and also related to Tx operations.
													//   This parameter can be a value of @ref HAL_UART_StateTypeDef

		__IO UartStateTypeDef    	RxState;     	//!< UART state information related to Rx operations.
													//   This parameter can be a value of @ref HAL_UART_StateTypeDef

		__IO UartErrorTypeDef       ErrorCode;      //!< UART Error code

	} UartHandle;

	#define UART_HWCONTROL_NONE                  0x00000000U                    			      //!< No hardware control
	#define UART_HWCONTROL_RTS                   ((uint32_t)USART_CR3_RTSE)                       //!< Request To Send
	#define UART_HWCONTROL_CTS                   ((uint32_t)USART_CR3_CTSE)                       //!< Clear To Send
	#define UART_HWCONTROL_RTS_CTS               ((uint32_t)(USART_CR3_RTSE | USART_CR3_CTSE))    //!< Request and Clear To Send

	//       Elements values convention: 0000ZZZZ00000000b
	//           - ZZZZ  : Flag position in the ISR register(4bits)
	//
	#define UART_IT_ORE                         0x0300U                  //!< UART overrun error interruption
	#define UART_IT_NE                          0x0200U                  //!< UART noise error interruption
	#define UART_IT_FE                          0x0100U                  //!< UART frame error interruption

	constexpr uint32_t UART_AUTOBAUD_REQUEST 		= USART_RQR_ABRRQ;        //!< Auto-Baud Rate Request
	constexpr uint32_t UART_SENDBREAK_REQUEST   	= USART_RQR_SBKRQ;        //!< Send Break Request
	constexpr uint32_t UART_MUTE_MODE_REQUEST   	= USART_RQR_MMRQ;         //!< Mute Mode Request
	constexpr uint32_t UART_RXDATA_FLUSH_REQUEST	= USART_RQR_RXFRQ;        //!< Receive Data flush Request
	constexpr uint32_t UART_TXDATA_FLUSH_REQUEST    = USART_RQR_TXFRQ;        //!< Transmit data flush Request

	#define UART_IT_MASK                        0x0000001FU                 //!< UART interruptions flags mask

	constexpr uint32_t UART_ADVFEATURE_NO_INIT 					= 0x00000000U;         //!< No advanced feature initialization
	constexpr uint32_t UART_ADVFEATURE_TXINVERT_INIT 			= 0x00000001U;         //!< TX pin active level inversion
	constexpr uint32_t UART_ADVFEATURE_RXINVERT_INIT 			= 0x00000002U;         //!< RX pin active level inversion
	constexpr uint32_t UART_ADVFEATURE_DATAINVERT_INIT 			= 0x00000004U;         //!< Binary data inversion
	constexpr uint32_t UART_ADVFEATURE_SWAP_INIT 				= 0x00000008U;         //!< TX/RX pins swap
	constexpr uint32_t UART_ADVFEATURE_RXOVERRUNDISABLE_INIT 	= 0x00000010U;         //!< RX overrun disable
	constexpr uint32_t UART_ADVFEATURE_DMADISABLEONERROR_INIT	= 0x00000020U;         //!< DMA disable on Reception Error
	constexpr uint32_t UART_ADVFEATURE_AUTOBAUDRATE_INIT 		= 0x00000040U;         //!< Auto Baud rate detection initialization
	constexpr uint32_t UART_ADVFEATURE_MSBFIRST_INIT 			= 0x00000080U;         //!< Most significant bit sent/received first

	constexpr uint32_t UART_ADVFEATURE_AUTOBAUDRATE_DISABLE 	= 0x00000000U;         //!< RX Auto Baud rate detection enable
	constexpr uint32_t UART_ADVFEATURE_AUTOBAUDRATE_ENABLE 		= USART_CR2_ABREN;     //!< RX Auto Baud rate detection disable

	#define UART_CR1_FIELDS  ((uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | \
										 USART_CR1_TE | USART_CR1_RE | USART_CR1_OVER8)) //!< UART or USART CR1 fields of parameters set by UART_SetConfig API

	#define UART_LPUART_BRR_MIN           0x00000300U  // LPUART BRR minimum authorized value
	#define UART_LPUART_BRR_MAX           0x000FFFFFU  // LPUART BRR maximum authorized value

	constexpr uint32_t UART_OVERSAMPLING_16 = 0x00000000U;         //!< Oversampling by 16
	constexpr uint32_t UART_OVERSAMPLING_8  = USART_CR1_OVER8;     //!< Oversampling by 8

	constexpr uint32_t HAL_UART_TIMEOUT_VALUE = 0x1FFFFFFU;                           //!< UART polling-based communications time-out value

	#define UART_WORDLENGTH_7B                  ((uint32_t)USART_CR1_M1)   					//!< 7-bit long UART frame
	#define UART_WORDLENGTH_8B                  0x00000000U				    				//!< 8-bit long UART frame
	#define UART_WORDLENGTH_9B                  ((uint32_t)USART_CR1_M0)   					//!< 9-bit long UART frame

	#define UART_STOPBITS_1                     0x00000000U	               					//!< UART frame with 1 stop bit
	#define UART_STOPBITS_1_5                   (USART_CR2_STOP_0 | USART_CR2_STOP_1) 		//!< UART frame with 1.5 stop bits
	#define UART_STOPBITS_2                      USART_CR2_STOP_1							//!< UART frame with 2 stop bits

	#define UART_PARITY_NONE                    0x00000000U 			                    //!< No parity
	#define UART_PARITY_EVEN                    ((uint32_t)USART_CR1_PCE)                   //!< Even parity
	#define UART_PARITY_ODD                     ((uint32_t)(USART_CR1_PCE | USART_CR1_PS))	//!< Odd parity

	#define UART_MODE_RX                        ((uint32_t)USART_CR1_RE)                    //!< RX mode
	#define UART_MODE_TX                        ((uint32_t)USART_CR1_TE)                    //!< TX mode
	#define UART_MODE_TX_RX                     ((uint32_t)(USART_CR1_TE |USART_CR1_RE))    //!< RX and TX mode

	#define UART_ONE_BIT_SAMPLE_DISABLE         0x00000000U							        //!< One-bit sampling disable
	#define UART_ONE_BIT_SAMPLE_ENABLE          ((uint32_t)USART_CR3_ONEBIT)    			//!< One-bit sampling enable

	// @defgroup UART_Interrupt_definition   UART Interrupts Definition
	//        Elements values convention: 000ZZZZZ0XXYYYYYb
	//           - YYYYY  : Interrupt source position in the XX register (5bits)
	//           - XX  : Interrupt source register (2bits)
	//                 - 01: CR1 register
	//                 - 10: CR2 register
	//                 - 11: CR3 register
	//           - ZZZZZ  : Flag position in the ISR register(5bits)
	//
	#define UART_IT_PE                          0x00000028U		//!< UART parity error interruption
	#define UART_IT_TXE                         0x00000727U     //!< UART transmit data register empty interruption
	#define UART_IT_TC                          0x00000626U     //!< UART transmission complete interruption
	#define UART_IT_RXNE                        0x00000525U 	//!< UART read data register not empty interruption
	#define UART_IT_IDLE                        0x00000424U     //!< UART idle interruption
	#define UART_IT_LBD                         0x00000846U     //!< UART LIN break detection interruption
	#define UART_IT_CTS                         0x0000096AU     //!< UART CTS interruption
	#define UART_IT_CM                          0x0000112EU     //!< UART character match interruption
	#define UART_IT_WUF                         0x00001476U     //!< UART wake-up from stop mode interruption

	#define UART_MAX_DELAY      0xFFFFFFFFU

	// @brief  BRR division operation to set BRR register in 8-bit oversampling mode.
	// @param  __PCLK__: UART clock.
	// @param  __BAUD__: Baud rate set by the user.
	// @retval Division result
	//
	#define UART_DIV_SAMPLING8(__PCLK__, __BAUD__)   ((((__PCLK__)*2U) + ((__BAUD__)/2U)) / (__BAUD__))

	// @brief  Check whether or not UART instance is Low Power UART.
	// @param  __HANDLE__: specifies the UART Handle.
	// @retval SET (instance is LPUART) or RESET (instance isn't LPUART)
	//
	#define UART_INSTANCE_LOWPOWER(__HANDLE__) (((__HANDLE__)->Instance == LPUART1) ? SET : RESET )

	// Define constants and variables for buffering incoming serial data.  We're
	// using a ring buffer (I think), in which rx_buffer_head is the index of the
	// location to which to write the next incoming character and rx_buffer_tail
	// is the index of the location from which to read.
	// 256 is the max limit due to uint8_t head and tail. Use only powers of 2. (...,16,32,64,128,256)
	#ifndef RX_BUFFER_SIZE
		#define RX_BUFFER_SIZE 128U
	#endif
	#ifndef TX_BUFFER_SIZE
		#define TX_BUFFER_SIZE 32U
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

	//
	// Private variables
	//

	static UartHandle hlpuart1;
	static UartHandle huart2;

	static ring_buffer_r rx_buffer = { { 0U }, 0U, 0U };
	#if TX_BUFFER_SIZE > 0U
		static ring_buffer_t tx_buffer = { { 0U }, 0U, 0U };
	#endif

	//
	// Private function declarations
	//

	// @brief  Disable the specified UART interrupt.
	// @param  __HANDLE__: specifies the UART Handle.
	// @param  __INTERRUPT__: specifies the UART interrupt source to disable.
	//          This parameter can be one of the following values:
	//            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
	//            @arg @ref UART_IT_CM   Character match interrupt
	//            @arg @ref UART_IT_CTS  CTS change interrupt
	//            @arg @ref UART_IT_LBD  LIN Break detection interrupt
	//            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
	//            @arg @ref UART_IT_TC   Transmission complete interrupt
	//            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
	//            @arg @ref UART_IT_IDLE Idle line detection interrupt
	//            @arg @ref UART_IT_PE   Parity Error interrupt
	//            @arg @ref UART_IT_ERR  Error interrupt (Frame error, noise error, overrun error)
	// @retval None
	//
	static void UartDisableIT(UartHandle const* const huart, uint32_t const source);

	// @brief  Enable the specified UART interrupt.
	// @param  __HANDLE__: specifies the UART Handle.
	// @param  __INTERRUPT__: specifies the UART interrupt source to enable.
	//          This parameter can be one of the following values:
	//            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
	//            @arg @ref UART_IT_CM   Character match interrupt
	//            @arg @ref UART_IT_CTS  CTS change interrupt
	//            @arg @ref UART_IT_LBD  LIN Break detection interrupt
	//            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
	//            @arg @ref UART_IT_TC   Transmission complete interrupt
	//            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
	//            @arg @ref UART_IT_IDLE Idle line detection interrupt
	//            @arg @ref UART_IT_PE   Parity Error interrupt
	//            @arg @ref UART_IT_ERR  Error interrupt (Frame error, noise error, overrun error)
	// @retval None
	//
	static void UartEnableIT(UartHandle const* const huart, uint32_t const source);

	// @brief  Check whether the specified UART interrupt source is enabled or not.
	// @param  __HANDLE__: specifies the UART Handle.
	// @param  __IT__: specifies the UART interrupt source to check.
	//          This parameter can be one of the following values:
	//            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
	//            @arg @ref UART_IT_CM   Character match interrupt
	//            @arg @ref UART_IT_CTS  CTS change interrupt
	//            @arg @ref UART_IT_LBD  LIN Break detection interrupt
	//            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
	//            @arg @ref UART_IT_TC   Transmission complete interrupt
	//            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
	//            @arg @ref UART_IT_IDLE Idle line detection interrupt
	//            @arg @ref UART_IT_ERR  Error interrupt (Frame error, noise error, overrun error)
	//            @arg @ref UART_IT_PE   Parity Error interrupt
	// @retval The new state of __IT__ (TRUE or FALSE).
	//
	static ITStatus UartGetITSource(UartHandle const* const huart, uint32_t const source);

	// @brief  Check whether the specified UART interrupt has occurred or not.
	// @param  __HANDLE__: specifies the UART Handle.
	// @param  __IT__: specifies the UART interrupt to check.
	//          This parameter can be one of the following values:
	//            @arg @ref UART_IT_WUF  Wakeup from stop mode interrupt
	//            @arg @ref UART_IT_CM   Character match interrupt
	//            @arg @ref UART_IT_CTS  CTS change interrupt
	//            @arg @ref UART_IT_LBD  LIN Break detection interrupt
	//            @arg @ref UART_IT_TXE  Transmit Data Register empty interrupt
	//            @arg @ref UART_IT_TC   Transmission complete interrupt
	//            @arg @ref UART_IT_RXNE Receive Data register not empty interrupt
	//            @arg @ref UART_IT_IDLE Idle line detection interrupt
	//            @arg @ref UART_IT_ORE  Overrun Error interrupt
	//            @arg @ref UART_IT_NE   Noise Error interrupt
	//            @arg @ref UART_IT_FE   Framing Error interrupt
	//            @arg @ref UART_IT_PE   Parity Error interrupt
	// @retval The new state of __IT__ (TRUE or FALSE).
	//
	static ITStatus UartGetIT(UartHandle const* const huart, uint32_t const source);

	// @brief  Set a specific UART request flag.
	// @param  __HANDLE__: specifies the UART Handle.
	// @param  __REQ__: specifies the request flag to set
	//          This parameter can be one of the following values:
	//            @arg @ref UART_AUTOBAUD_REQUEST Auto-Baud Rate Request
	//            @arg @ref UART_SENDBREAK_REQUEST Send Break Request
	//            @arg @ref UART_MUTE_MODE_REQUEST Mute Mode Request
	//            @arg @ref UART_RXDATA_FLUSH_REQUEST Receive Data flush Request
	//            @arg @ref UART_TXDATA_FLUSH_REQUEST Transmit data flush Request
	// @retval None
	//
	static void UartSendReq(UartHandle const* const huart, uint32_t const req);

	static void UartMspInit(UartHandle const* const huart);

	static void UART_AdvFeatureConfig(UartHandle const* const huart);

	static StatusTypeDef UartSetConfig(UartHandle* huart);

	//
	// @brief  Handle UART Communication Timeout.
	// @param  huart UART handle.
	// @param  Flag Specifies the UART flag to check
	// @param  Status Flag status (SET or RESET)
	// @param  Tickstart Tick start value
	// @param  Timeout Timeout duration
	// @retval Status
	//
	static StatusTypeDef UART_WaitOnFlagUntilTimeout(
		UartHandle* const huart,
		uint32_t const Flag,
		FlagStatus const Status,
		uint32_t const Tickstart,
		uint32_t const Timeout
	);

	//
	// @brief Check the UART Idle State.
	// @param huart UART handle.
	// @retval HAL status
	//
	static StatusTypeDef UART_CheckIdleState(UartHandle* const huart);

	//
	// @brief Initialize the UART mode according to the specified
	//        parameters in the UART_InitTypeDef and initialize the associated handle.
	// @param huart: UART handle.
	// @retval HAL status
	//
	static StatusTypeDef UartInit(UartHandle* const huart);

	//
	// @brief DeInitialize the UART peripheral.
	// @param huart: UART handle.
	// @retval HAL status
	//
	static StatusTypeDef UartDeInit(UartHandle* const huart);

	// LPUART1 init function
	static void MX_LPUART1_UART_Init(uint32_t const baud);

	// USART2 init function
	static void MX_USART2_UART_Init(uint32_t const baud);

	static void UartMspDeInit(UartHandle const* const huart);

	static void transmit(UartHandle const* const huart, uint8_t const byte);

	static void printNumber(uint32_t n, uint8_t const base);

	static void print(char const c, int32_t const base = BYTE);

	static void print(uint8_t const b, int32_t const base = BYTE);

	// @brief  Disable UART.
	// @param  __HANDLE__: specifies the UART Handle.
	// @retval None
	//
	static FORCE_INLINE void UART_Disable(UartHandle* huart);

	/** @brief  Enable UART.
	  * @param  __HANDLE__: specifies the UART Handle.
	  * @retval None
	  */
	static FORCE_INLINE void UART_Enable(UartHandle* huart);

	// @brief  Check whether the specified UART flag is set or not.
	// @param  __HANDLE__: specifies the UART Handle.
	// @param  __FLAG__: specifies the flag to check.
	//        This parameter can be one of the following values:
	//            @arg @ref UART_FLAG_REACK Receive enable acknowledge flag
	//            @arg @ref UART_FLAG_TEACK Transmit enable acknowledge flag
	//            @arg @ref UART_FLAG_WUF   Wake up from stop mode flag
	//            @arg @ref UART_FLAG_RWU   Receiver wake up flag (if the UART in mute mode)
	//            @arg @ref UART_FLAG_SBKF  Send Break flag
	//            @arg @ref UART_FLAG_CMF   Character match flag
	//            @arg @ref UART_FLAG_BUSY  Busy flag
	//            @arg @ref UART_FLAG_ABRF  Auto Baud rate detection flag
	//            @arg @ref UART_FLAG_ABRE  Auto Baud rate detection error flag
	//            @arg @ref UART_FLAG_EOBF  End of block flag
	//            @arg @ref UART_FLAG_RTOF  Receiver timeout flag
	//            @arg @ref UART_FLAG_CTS   CTS Change flag
	//            @arg @ref UART_FLAG_LBDF  LIN Break detection flag
	//            @arg @ref UART_FLAG_TXE   Transmit data register empty flag
	//            @arg @ref UART_FLAG_TC    Transmission Complete flag
	//            @arg @ref UART_FLAG_RXNE  Receive data register not empty flag
	//            @arg @ref UART_FLAG_IDLE  Idle Line detection flag
	//            @arg @ref UART_FLAG_ORE   Overrun Error flag
	//            @arg @ref UART_FLAG_NE    Noise Error flag
	//            @arg @ref UART_FLAG_FE    Framing Error flag
	//            @arg @ref UART_FLAG_PE    Parity Error flag
	// @retval The new state of __FLAG__ (TRUE or FALSE).
	//
	static FORCE_INLINE FlagStatus UART_GetFlagStatus(UartHandle* huart, uint32_t flag);

	static FORCE_INLINE void UART_UnlockHandle(UartHandle* huart);

	// @brief  BRR division operation to set BRR register in 16-bit oversampling mode.
	// @param  __PCLK__: UART clock.
	// @param  __BAUD__: Baud rate set by the user.
	// @retval Division result
	//
	static FORCE_INLINE uint32_t UART_DivSampling16(uint32_t pclk, uint32_t baud);

	// @brief  BRR division operation to set BRR register with LPUART.
	// @param  __PCLK__: LPUART clock.
	// @param  __BAUD__: Baud rate set by the user.
	// @retval Division result
	//
	static FORCE_INLINE uint32_t UART_DivLpUart(uint32_t pclk, uint32_t baud);

	static FORCE_INLINE void store_char(uint8_t const c);

    static FORCE_INLINE void write(const char* str);
    static FORCE_INLINE void write(uint8_t const* buffer, size_t size);

    //
	// Public variable initialization
	//

	//
	// Namespace body
	//

	static FORCE_INLINE UART_ClockSourceTypeDef UART_GetClockSource(UartHandle* huart) {
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

	void flush(void) {
		// RX
		// don't reverse this or there may be problems if the RX interrupt
		// occurs after reading the value of rx_buffer_head but before writing
		// the value to rx_buffer_tail; the previous value of rx_buffer_head
		// may be written to rx_buffer_tail, making it appear as if the buffer
		// were full, not empty.
		__disable_irq();
		rx_buffer.head = rx_buffer.tail;
		__enable_irq();
	}

	uint8_t available(void) {
		__disable_irq();
		const uint8_t h = rx_buffer.head;
		const uint8_t t = rx_buffer.tail;
		__enable_irq();
		return ((RX_BUFFER_SIZE + h) - t) & static_cast<uint8_t>(RX_BUFFER_SIZE - 1U);
	}

	int32_t read(void) {
		int32_t v;
		__disable_irq();
		const uint8_t t = rx_buffer.tail;
		if (rx_buffer.head == t) {
			v = -1;
		} else {
			v = static_cast<int32_t>(rx_buffer.buffer[t]);
			rx_buffer.tail = (t + 1U) & static_cast<uint8_t>(RX_BUFFER_SIZE - 1U);
		}
		__enable_irq();
		return v;
	}

	static void UartMspInit(UartHandle const* const huart)
	{
		GPIO::GpioInit_t GPIO_InitStruct;
		//lint -save -e1924 -e9078 -e923 -e1960 -e835 -e9053
		if(huart->Instance == LPUART1)
		{
			// Peripheral clock enable
			RCC_LPUART1_CLK_ENABLE();

			// LPUART1 GPIO Configuration
			//    PA6     ------> LPUART1_CTS
			// PC4     ------> LPUART1_TX
			// PC5     ------> LPUART1_RX
			//    PB1     ------> LPUART1_RTS
			//
			//    GPIO_InitStruct.Pin = GPIO_PIN_6;
			//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			//    GPIO_InitStruct.Pull = GPIO_NOPULL;
			//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			//    GPIO_InitStruct.Alternate = GPIO_AF4_LPUART1;
			//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

			GPIO_InitStruct.Pin = static_cast<uint32_t>(GPIO::GPIO_PIN_4 | GPIO::GPIO_PIN_5);
			GPIO_InitStruct.Mode = GPIO::GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.Alternate = GPIO_AF2_LPUART1;
			GPIO::GpioInit(GPIOC, &GPIO_InitStruct);

			//    GPIO_InitStruct.Pin = GPIO_PIN_1;
			//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
			//    GPIO_InitStruct.Pull = GPIO_NOPULL;
			//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			//    GPIO_InitStruct.Alternate = GPIO_AF4_LPUART1;
			//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

			// LPUART1 interrupt Init
			NVIC_SetPriority(AES_RNG_LPUART1_IRQn, 0U);
			NVIC_EnableIRQ(AES_RNG_LPUART1_IRQn);
		}
		else if(huart->Instance == USART2)
		{
			// Peripheral clock enable
			RCC_USART2_CLK_ENABLE();

			// USART2 GPIO Configuration
			// PA2     ------> USART2_TX
			// PA3     ------> USART2_RX
			//
			GPIO_InitStruct.Pin = static_cast<uint32_t>(GPIO::GPIO_PIN_2 | GPIO::GPIO_PIN_3);
			GPIO_InitStruct.Mode = GPIO::GPIO_MODE_AF_PP;
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
			GPIO::GpioInit(GPIOA, &GPIO_InitStruct);

			// USART2 interrupt Init
			NVIC_SetPriority(USART2_IRQn, 0U);
			NVIC_EnableIRQ(USART2_IRQn);
		} else {
			// Should not get here
		}
	}

	//
	// @brief Configure the UART peripheral advanced features.
	// @param huart: UART handle.
	// @retval None
	static void UART_AdvFeatureConfig(UartHandle const* const huart)
	{
		//lint -save -e9033 -e9032
		// if required, configure TX pin active level inversion
		if(isBitSet(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_TXINVERT_INIT))
		{
			MODIFY_REG(huart->Instance->CR2, USART_CR2_TXINV, huart->AdvancedInit.TxPinLevelInvert);
		}

		// if required, configure RX pin active level inversion
		if(isBitSet(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_RXINVERT_INIT))
		{
			MODIFY_REG(huart->Instance->CR2, USART_CR2_RXINV, huart->AdvancedInit.RxPinLevelInvert);
		}

		// if required, configure data inversion
		if(isBitSet(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_DATAINVERT_INIT))
		{
			MODIFY_REG(huart->Instance->CR2, USART_CR2_DATAINV, huart->AdvancedInit.DataInvert);
		}

		// if required, configure RX/TX pins swap
		if(isBitSet(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_SWAP_INIT))
		{
			MODIFY_REG(huart->Instance->CR2, USART_CR2_SWAP, huart->AdvancedInit.Swap);
		}

		// if required, configure RX overrun detection disabling
		if(isBitSet(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_RXOVERRUNDISABLE_INIT))
		{
			MODIFY_REG(huart->Instance->CR3, USART_CR3_OVRDIS, huart->AdvancedInit.OverrunDisable);
		}

		// if required, configure DMA disabling on reception error
		if(isBitSet(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_DMADISABLEONERROR_INIT))
		{
			MODIFY_REG(huart->Instance->CR3, USART_CR3_DDRE, huart->AdvancedInit.DMADisableonRxError);
		}

		// if required, configure auto Baud rate detection scheme
		if(isBitSet(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_AUTOBAUDRATE_INIT))
		{
			MODIFY_REG(huart->Instance->CR2, USART_CR2_ABREN, huart->AdvancedInit.AutoBaudRateEnable);
			// set auto Baudrate detection parameters if detection is enabled
			if(huart->AdvancedInit.AutoBaudRateEnable == UART_ADVFEATURE_AUTOBAUDRATE_ENABLE)
			{
				MODIFY_REG(huart->Instance->CR2, USART_CR2_ABRMODE, huart->AdvancedInit.AutoBaudRateMode);
			}
		}

		// if required, configure MSB first on communication line
		if(isBitSet(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_MSBFIRST_INIT))
		{
			MODIFY_REG(huart->Instance->CR2, USART_CR2_MSBFIRST, huart->AdvancedInit.MSBFirst);
		}
		//lint -restore
	}

	//
	// @brief Configure the UART peripheral.
	// @param huart: UART handle.
	// @retval HAL status
	//
	static StatusTypeDef UartSetConfig(UartHandle* const huart)
	{
		uint16_t brrtemp                    = 0x0000U;
		uint16_t usartdiv                   = 0x0000U;
		StatusTypeDef ret               = STATUS_OK;

		//-------------------------- USART CR1 Configuration -----------------------
		// Clear M, PCE, PS, TE, RE and OVER8 bits and configure
		// the UART Word Length, Parity, Mode and oversampling:
		// set the M bits according to huart->Init.WordLength value
		// set PCE and PS bits according to huart->Init.Parity value
		// set TE and RE bits according to huart->Init.Mode value
		// set OVER8 bit according to huart->Init.OverSampling value
		uint32_t tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode | huart->Init.OverSampling ;
		//lint -save -e9032
		MODIFY_REG(huart->Instance->CR1, UART_CR1_FIELDS, tmpreg);

		//-------------------------- USART CR2 Configuration -----------------------
		// Configure the UART Stop Bits: Set STOP[13:12] bits according
		// to huart->Init.StopBits value
		MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits);
		//lint -restore

		//-------------------------- USART CR3 Configuration -----------------------
		// Configure
		// - UART HardWare Flow Control: set CTSE and RTSE bits according
		//   to huart->Init.HwFlowCtl value
		// - one-bit sampling method versus three samples' majority rule according
		//   to huart->Init.OneBitSampling (not applicable to LPUART) */
		tmpreg = (uint32_t)huart->Init.HwFlowCtl;
		if (UART_INSTANCE_LOWPOWER(huart) == RESET)
		{
			tmpreg |= huart->Init.OneBitSampling;
		}
		MODIFY_REG(huart->Instance->CR3, (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT), tmpreg);

		//-------------------------- USART BRR Configuration -----------------------
		UART_ClockSourceTypeDef const clocksource = UART_GetClockSource(huart);
		const uint32_t frequency = Rcc::RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_USART2);

		// Check LPUART instance
		if(UART_INSTANCE_LOWPOWER(huart) != RESET)
		{
			// Retrieve frequency clock
			tmpreg = 0;

			switch (clocksource)
			{
				case UART_CLOCKSOURCE_PCLK1:
					tmpreg = Rcc::RCC_GetPCLK1Freq();
					break;
				case UART_CLOCKSOURCE_HSI:
					if (Rcc::RCC_GetFlag(Rcc::RCC_FLAG_HSIDIV) != 0U)
					{
						tmpreg = (uint32_t) (HSI_VALUE >> 2U);
					}
					else
					{
						tmpreg = (uint32_t) HSI_VALUE;
					}
					break;
				case UART_CLOCKSOURCE_SYSCLK:
					tmpreg = Rcc::RCC_GetSysClockFreq();
					break;
				case UART_CLOCKSOURCE_LSE:
					tmpreg = (uint32_t) LSE_VALUE;
					break;
				case UART_CLOCKSOURCE_UNDEFINED:
				case UART_CLOCKSOURCE_PCLK2:
				default:
					ret = STATUS_ERROR;
					break;
			}

			// if proper clock source reported
			if (tmpreg != 0U)
			{
				// ensure that Frequency clock is in the range [3 * baudrate, 4096 * baudrate]
				if ( (tmpreg < (3U * huart->Init.BaudRate) ) ||
						(tmpreg > (4096U * huart->Init.BaudRate) ))
				{
					ret = STATUS_ERROR;
				}
				else
				{
					tmpreg = (uint32_t)(UART_DivLpUart(tmpreg, huart->Init.BaudRate));

					if ((tmpreg >= UART_LPUART_BRR_MIN) && (tmpreg <= UART_LPUART_BRR_MAX))
					{
						huart->Instance->BRR = tmpreg;
					}
					else
					{
						ret = STATUS_ERROR;
					}
				}  //   if ( (tmpreg < (3 * huart->Init.BaudRate) ) || (tmpreg > (4096 * huart->Init.BaudRate) ))
			} // if (tmpreg != 0)
		}
		// Check UART Over Sampling to set Baud Rate Register
		else if (huart->Init.OverSampling == UART_OVERSAMPLING_8)
		{
			switch (clocksource)
			{
				case UART_CLOCKSOURCE_PCLK1:
					usartdiv = (uint16_t)(UART_DIV_SAMPLING8(frequency, huart->Init.BaudRate));
					break;
				case UART_CLOCKSOURCE_PCLK2:
					usartdiv = (uint16_t)(UART_DIV_SAMPLING8(Rcc::RCC_GetPCLK2Freq(), huart->Init.BaudRate));
					break;
				case UART_CLOCKSOURCE_HSI:
					if (Rcc::RCC_GetFlag(Rcc::RCC_FLAG_HSIDIV) != 0U)
					{
						usartdiv = (uint16_t)(UART_DIV_SAMPLING8((HSI_VALUE >> 2U), huart->Init.BaudRate));
					}
					else
					{
						usartdiv = (uint16_t)(UART_DIV_SAMPLING8(HSI_VALUE, huart->Init.BaudRate));
					}
					break;
				case UART_CLOCKSOURCE_SYSCLK:
					usartdiv = (uint16_t)(UART_DIV_SAMPLING8(Rcc::RCC_GetSysClockFreq(), huart->Init.BaudRate));
					break;
				case UART_CLOCKSOURCE_LSE:
					usartdiv = (uint16_t)(UART_DIV_SAMPLING8(LSE_VALUE, huart->Init.BaudRate));
					break;
				case UART_CLOCKSOURCE_UNDEFINED:
				default:
					ret = STATUS_ERROR;
					break;
			}

			brrtemp = usartdiv & 0xFFF0U;
			brrtemp |= (uint16_t)((uint16_t)(usartdiv & (uint16_t)0x000FU) >> (uint16_t)1U);
			huart->Instance->BRR = brrtemp;
		}
		else
		{
			switch (clocksource)
			{
				case UART_CLOCKSOURCE_PCLK1:
					huart->Instance->BRR = UART_DivSampling16(Rcc::RCC_GetPCLK1Freq(), huart->Init.BaudRate);
					break;
				case UART_CLOCKSOURCE_PCLK2:
					huart->Instance->BRR = UART_DivSampling16(Rcc::RCC_GetPCLK2Freq(), huart->Init.BaudRate);
					break;
				case UART_CLOCKSOURCE_HSI:
					if (Rcc::RCC_GetFlag(Rcc::RCC_FLAG_HSIDIV) != 0U)
					{
						huart->Instance->BRR = UART_DivSampling16((HSI_VALUE >> 2U), huart->Init.BaudRate);
					}
					else
					{
						huart->Instance->BRR = UART_DivSampling16(HSI_VALUE, huart->Init.BaudRate);
					}
					break;
				case UART_CLOCKSOURCE_SYSCLK:
					huart->Instance->BRR = UART_DivSampling16(Rcc::RCC_GetSysClockFreq(), huart->Init.BaudRate);
					break;
				case UART_CLOCKSOURCE_LSE:
					huart->Instance->BRR = UART_DivSampling16(LSE_VALUE, huart->Init.BaudRate);
					break;
				case UART_CLOCKSOURCE_UNDEFINED:
				default:
					ret = STATUS_ERROR;
					break;
			}
		}

		return ret;
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
	static StatusTypeDef UART_WaitOnFlagUntilTimeout(
		UartHandle* const huart,
		uint32_t const Flag,
		FlagStatus const Status,
		uint32_t const Tickstart,
		uint32_t const Timeout
	) {
		StatusTypeDef res = STATUS_OK;
		// Wait until flag is set
		while(Status == UART_GetFlagStatus(huart, Flag))
		{
			// Check for the Timeout
			if(Timeout != UART_MAX_DELAY)
			{
				uint32_t const elapsedTime = Timers::GetTick() - Tickstart;
				if((Timeout == 0U) || (elapsedTime > Timeout))
				{
					// Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process
					CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE));
					constexpr uint8_t USART_CR3_EIE_BIT = USART_CR3_EIE;
					CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE_BIT);

					huart->gState  = UART_STATE_READY;
					huart->RxState = UART_STATE_READY;

					// Process Unlocked
					UART_UnlockHandle(huart);
					res = STATUS_TIMEOUT;
					break;
				}
			}
		}
		return res;
	}

	//
	// @brief Check the UART Idle State.
	// @param huart UART handle.
	// @retval HAL status
	//
	static StatusTypeDef UART_CheckIdleState(UartHandle* const huart)
	{
		StatusTypeDef res = STATUS_OK;

		// Initialize the UART ErrorCode
		huart->ErrorCode = UART_ERROR_NONE;

		// Init tickstart for timeout managment
		uint32_t const tickstart = Timers::GetTick();

		// Check if the Transmitter is enabled
		constexpr uint8_t USART_CR1_TE_BIT = USART_CR1_TE;
		if((huart->Instance->CR1 & USART_CR1_TE_BIT) == USART_CR1_TE_BIT)
		{
			// Wait until TEACK flag is set
			//lint -save -e9031
			// some error in lint
			constexpr uint32_t USART_ISR_TEACK_BIT = USART_ISR_TEACK;
			//lint -restore
			if(UART_WaitOnFlagUntilTimeout(huart, USART_ISR_TEACK_BIT, RESET, tickstart, HAL_UART_TIMEOUT_VALUE) != STATUS_OK)
			{
				// Timeout occurred
				res = STATUS_TIMEOUT;
			}
		}
		// Check if the Receiver is enabled
		constexpr uint8_t USART_CR1_RE_BIT = USART_CR1_RE;
		if (res == STATUS_OK) {
			if ((huart->Instance->CR1 & USART_CR1_RE_BIT) == USART_CR1_RE_BIT)
			{
				// Wait until REACK flag is set
				//lint -save -e9031
				// error in lint
				constexpr uint32_t USART_ISR_REACK_BIT = USART_ISR_REACK;
				//lint -restore
				if(UART_WaitOnFlagUntilTimeout(huart, USART_ISR_REACK_BIT, RESET, tickstart, HAL_UART_TIMEOUT_VALUE) != STATUS_OK)
				{
					// Timeout occurred
					res = STATUS_TIMEOUT;
				}
			}
		}

		if (res == STATUS_OK) {
			// Initialize the UART State
			huart->gState  = UART_STATE_READY;
			huart->RxState = UART_STATE_READY;

			// Process Unlocked
			UART_UnlockHandle(huart);
		}

		return res;
	}

	//
	// @brief Initialize the UART mode according to the specified
	//        parameters in the UART_InitTypeDef and initialize the associated handle.
	// @param huart: UART handle.
	// @retval HAL status
	//
	static StatusTypeDef UartInit(UartHandle* const huart)
	{
		StatusTypeDef res;
		// Check the UART handle allocation
		if(huart == NULL)
		{
			res = STATUS_ERROR;
		}
		else
		{
			if(huart->gState == UART_STATE_RESET)
			{
				// Allocate lock resource and initialize it
				huart->Lock = HANDLE_UNLOCKED;

				// Init the low level hardware : GPIO, CLOCK
				UartMspInit(huart);
			}

			huart->gState = UART_STATE_BUSY;

			// Disable the Peripheral
			UART_Disable(huart);

			// Set the UART Communication parameters
			if (huart->AdvancedInit.AdvFeatureInit != UART_ADVFEATURE_NO_INIT)
			{
				UART_AdvFeatureConfig(huart);
			}

			if (UartSetConfig(huart) == STATUS_ERROR)
			{
				res = STATUS_ERROR;
			}
			else
			{

				// In asynchronous mode, the following bits must be kept cleared:
				// - LINEN and CLKEN bits in the USART_CR2 register,
				// - SCEN, HDSEL and IREN  bits in the USART_CR3 register.
				CLEAR_BIT(huart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
				CLEAR_BIT(huart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

				// Enable the Peripheral
				UART_Enable(huart);

				// TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready
				res = UART_CheckIdleState(huart);
			}
		}

		return res;
	}

	// LPUART1 init function
	static void MX_LPUART1_UART_Init(uint32_t const baud)
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
		if (UartInit(&hlpuart1) != STATUS_OK)
		{
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}
	}

	// USART2 init function
	static void MX_USART2_UART_Init(uint32_t const baud)
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
		if (UartInit(&huart2) != STATUS_OK)
		{
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}
	}

	void begin(const uint32_t baud) {

		MX_USART2_UART_Init(baud);
		//	MX_LPUART1_UART_Init(baud);

		UartEnableIT(&huart2, UART_IT_RXNE);
		//	UartEnableIT(&hlpuart1, UART_IT_RXNE);

	  	#if TX_BUFFER_SIZE > 0U
			UartEnableIT(&huart2, UART_IT_TXE);
			UartEnableIT(&huart2, UART_IT_TC);
			//	UartEnableIT(&hlpuart1, UART_IT_TXE);
			//	UartEnableIT(&hlpuart1, UART_IT_TC);
	  	#endif
	}

	static void UartMspDeInit(UartHandle const* const huart)
	{

		if(huart->Instance == LPUART1)
		{
			// Peripheral clock disable
			RCC_LPUART1_CLK_DISABLE();

			//LPUART1 GPIO Configuration
			//    PA6     ------> LPUART1_CTS
			//PC4     ------> LPUART1_TX
			//PC5     ------> LPUART1_RX
			//    PB1     ------> LPUART1_RTS
			//
			//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

			GPIO::GpioDeInit(GPIOC, GPIO::GPIO_PIN_4 | GPIO::GPIO_PIN_5);

			//    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

			// LPUART1 interrupt DeInit
			NVIC_DisableIRQ(AES_RNG_LPUART1_IRQn);
		}
		else if(huart->Instance == USART2)
		{
			// Peripheral clock disable
			RCC_USART2_CLK_DISABLE();

			//USART2 GPIO Configuration
			// PA2     ------> USART2_TX
			// PA3     ------> USART2_RX
			//
			GPIO::GpioDeInit(GPIOA, GPIO::GPIO_PIN_2 | GPIO::GPIO_PIN_3);

			// USART2 interrupt DeInit
			NVIC_DisableIRQ(USART2_IRQn);
		} else {
			// should not get here
		}
	}

	//
	// @brief DeInitialize the UART peripheral.
	// @param huart: UART handle.
	// @retval HAL status
	//
	static StatusTypeDef UartDeInit(UartHandle* const huart)
	{
		StatusTypeDef res;
		// Check the UART handle allocation
		if(huart == NULL)
		{
			res = STATUS_ERROR;
		}
		else
		{

			huart->gState = UART_STATE_BUSY;

			// Disable the Peripheral
			UART_Disable(huart);

			huart->Instance->CR1 = 0x0U;
			huart->Instance->CR2 = 0x0U;
			huart->Instance->CR3 = 0x0U;

			// DeInit the low level hardware
			UartMspDeInit(huart);

			huart->ErrorCode = UART_ERROR_NONE;
			huart->gState    = UART_STATE_RESET;
			huart->RxState   = UART_STATE_RESET;

			// Process Unlock
			UART_UnlockHandle(huart);

			res = STATUS_OK;
		}

		return res;
	}

	void end() {
		if (UartDeInit(&huart2) != STATUS_OK) {
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}
		//	UartDeInit(&hlpuart1);
	}

	static void transmit(UartHandle const* const huart, uint8_t const byte)
	{
		__disable_irq();
		const uint8_t h = tx_buffer.head;
		const uint8_t i = (h + 1U) & static_cast<uint8_t>(TX_BUFFER_SIZE - 1);

		if (i != tx_buffer.tail) {
			tx_buffer.buffer[h] = byte;
			tx_buffer.head = i;
		}
		__enable_irq();
		UartEnableIT(huart, UART_IT_TXE);
	}

	void UartDisableIT(UartHandle const* const huart, uint32_t const source) {
		uint32_t const bitToClear = ((uint32_t)1U << (source & UART_IT_MASK));
		if (((source & 0xFFU) >> 5U) == 1U) {
			CLEAR_BIT(huart->Instance->CR1, bitToClear);
		}
		else if (((source & 0xFFU) >> 5U) == 2U) {
			CLEAR_BIT(huart->Instance->CR2, bitToClear);
		} else {
			CLEAR_BIT(huart->Instance->CR3, bitToClear);
		}
	}

	static void UartEnableIT(UartHandle const* const huart, uint32_t const source) {
		uint32_t const bitToSet = (uint32_t)1U << (source & UART_IT_MASK);
		if (((source & 0xFFU) >> 5U) == 1U) {
			SET_BIT(huart->Instance->CR1, bitToSet);
		} else if (((source & 0xFFU) >> 5U) == 2U) {
			SET_BIT(huart->Instance->CR2, bitToSet);
		} else {
			SET_BIT(huart->Instance->CR3, bitToSet);
		}
	}

	static ITStatus UartGetITSource(UartHandle const* const huart, uint32_t const source) {
		ITStatus res = RESET;
		uint32_t const srcMaskCheck = (uint32_t)1U << ((source & 0xFFFFU) & UART_IT_MASK);
		uint32_t reg;
		if (((source & 0xFFU) >> 5U) == 1U) {
			reg = huart->Instance->CR1;
		} else if (((source & 0xFFU) >> 5U) == 2U) {
			reg = huart->Instance->CR2;
		} else {
			reg = huart->Instance->CR3;
		}
		uint32_t const regSrc = reg & srcMaskCheck;
		if (regSrc > 0U) {
			res = SET;
		}
		return res;
	}

	static ITStatus UartGetIT(UartHandle const* const huart, uint32_t const source) {
		ITStatus res = RESET;
		if ((huart->Instance->ISR & ((uint32_t)1U << (source >> 0x08U))) > 0U) {
			res = SET;
		}
		return  res;
	}

	static void UartSendReq(UartHandle const* const huart, uint32_t const req) {
		(huart->Instance->RQR |= (uint32_t)req);
	}

	void write(const uint8_t c) {
		transmit(&huart2, c);
	}

	void print(uint32_t const n, int32_t const base) {
		if (base == 0) {
			write(static_cast<uint8_t>(n));
		} else {
			printNumber(n, static_cast<uint8_t>(base));
		}
	}

	static void printNumber(uint32_t n, uint8_t const base) {
		if (n != 0U) {
			uint8_t buf[8U * sizeof(uint32_t)]; // Enough space for base 2
			uint8_t i = 0U;
			while (n != 0U) {
				buf[i] = static_cast<uint8_t>(n % base);
				i++;
				n /= base;
			}
			while (i != 0U) {
				i--;
				const bool buf_i_lt_10 = buf[i] < 10U;
				int16_t const c0 = static_cast<int16_t>('0');
				const uint8_t c = static_cast<uint8_t>((buf_i_lt_10 ?
					static_cast<uint16_t>(c0) :
					static_cast<uint16_t>('A' - 10)) & 0x00FFU);
				print(static_cast<uint8_t>(buf[i] + c));
			}
		} else {
			print('0');
		}
	}

	void print(int32_t const n, int32_t const base) {
		int32_t num = n;
		if (base == 0) {
			write(static_cast<uint8_t>(num));
		} else {
			if (base == 10) {
				if (num < 0) {
					print('-');
					num = -num;
				}
			}
			printNumber(static_cast<uint32_t>(num), static_cast<uint8_t>(base));
		}
	}

	static void print(char const c, int32_t const base) {
		print(static_cast<int32_t>(c), base);
	}

	static void print(uint8_t const b, int32_t const base) {
		print(static_cast<uint32_t>(b), base);
	}

	// @brief  Disable UART.
	// @param  __HANDLE__: specifies the UART Handle.
	// @retval None
	//
	static FORCE_INLINE void UART_Disable(UartHandle* huart) {
		huart->Instance->CR1 &= ~(static_cast<uint32_t>(USART_CR1_UE));
	}

	/** @brief  Enable UART.
	  * @param  __HANDLE__: specifies the UART Handle.
	  * @retval None
	  */
	static FORCE_INLINE void UART_Enable(UartHandle* huart) {
		huart->Instance->CR1 |= static_cast<uint32_t>(USART_CR1_UE);
	}

	// @brief  Check whether the specified UART flag is set or not.
	// @param  __HANDLE__: specifies the UART Handle.
	// @param  __FLAG__: specifies the flag to check.
	//        This parameter can be one of the following values:
	//            @arg @ref UART_FLAG_REACK Receive enable acknowledge flag
	//            @arg @ref UART_FLAG_TEACK Transmit enable acknowledge flag
	//            @arg @ref UART_FLAG_WUF   Wake up from stop mode flag
	//            @arg @ref UART_FLAG_RWU   Receiver wake up flag (if the UART in mute mode)
	//            @arg @ref UART_FLAG_SBKF  Send Break flag
	//            @arg @ref UART_FLAG_CMF   Character match flag
	//            @arg @ref UART_FLAG_BUSY  Busy flag
	//            @arg @ref UART_FLAG_ABRF  Auto Baud rate detection flag
	//            @arg @ref UART_FLAG_ABRE  Auto Baud rate detection error flag
	//            @arg @ref UART_FLAG_EOBF  End of block flag
	//            @arg @ref UART_FLAG_RTOF  Receiver timeout flag
	//            @arg @ref UART_FLAG_CTS   CTS Change flag
	//            @arg @ref UART_FLAG_LBDF  LIN Break detection flag
	//            @arg @ref UART_FLAG_TXE   Transmit data register empty flag
	//            @arg @ref UART_FLAG_TC    Transmission Complete flag
	//            @arg @ref UART_FLAG_RXNE  Receive data register not empty flag
	//            @arg @ref UART_FLAG_IDLE  Idle Line detection flag
	//            @arg @ref UART_FLAG_ORE   Overrun Error flag
	//            @arg @ref UART_FLAG_NE    Noise Error flag
	//            @arg @ref UART_FLAG_FE    Framing Error flag
	//            @arg @ref UART_FLAG_PE    Parity Error flag
	// @retval The new state of __FLAG__ (TRUE or FALSE).
	//
	static FORCE_INLINE FlagStatus UART_GetFlagStatus(UartHandle* huart, uint32_t flag) {
		return ((huart->Instance->ISR & flag) == flag) ? SET : RESET;
	}

	static FORCE_INLINE void UART_UnlockHandle(UartHandle* huart) {
		huart->Lock = HANDLE_UNLOCKED;
	}

	// @brief  BRR division operation to set BRR register in 16-bit oversampling mode.
	// @param  __PCLK__: UART clock.
	// @param  __BAUD__: Baud rate set by the user.
	// @retval Division result
	//
	static FORCE_INLINE uint32_t UART_DivSampling16(uint32_t pclk, uint32_t baud) {
		return ((pclk + (baud / 2U)) / baud) & 0xFFFFU;
	}

	// @brief  BRR division operation to set BRR register with LPUART.
	// @param  __PCLK__: LPUART clock.
	// @param  __BAUD__: Baud rate set by the user.
	// @retval Division result
	//
	static FORCE_INLINE uint32_t UART_DivLpUart(uint32_t pclk, uint32_t baud) {
		return (((static_cast<uint64_t>(pclk) * 256U) + (baud / 2U)) / baud);
	}

	static FORCE_INLINE void store_char(uint8_t const c) {
		__disable_irq();
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
		__enable_irq();
	}

    void print(char const* str) {
    	write(str);
    }

    static FORCE_INLINE void write(char const* str) {
    	while (*str) {
    		write(*str++);
    	}
    }

    static FORCE_INLINE void write(uint8_t const* buffer, size_t size) {
    	while (size--) {
    		write(*buffer++);
    	}
    }

    // End

}

extern "C" {
	//
	// @brief This function handles AES, RNG and LPUART1 interrupts / LPUART1 wake-up interrupt through EXTI line 28.
	//
	void RNG_LPUART1_IRQHandler(void)
	{
		// USER CODE BEGIN AES_RNG_LPUART1_IRQn 0
		if (MarlinSerial::UartGetIT(&MarlinSerial::hlpuart1, UART_IT_RXNE) != RESET) {
			if (MarlinSerial::UartGetITSource(&MarlinSerial::hlpuart1, UART_IT_RXNE) != RESET)
			{
				// read only of no errors, else skip byte
				bool hasErrorBitSet = false;
				if (MarlinSerial::UartGetIT(&MarlinSerial::hlpuart1, UART_IT_NE) != RESET) {
					hasErrorBitSet = true;
				} else if (MarlinSerial::UartGetIT(&MarlinSerial::hlpuart1, UART_IT_FE) != RESET) {
					hasErrorBitSet = true;
				} else if (MarlinSerial::UartGetIT(&MarlinSerial::hlpuart1, UART_IT_PE) != RESET) {
					hasErrorBitSet = true;
				} else {
					hasErrorBitSet = false;
				}
				if (hasErrorBitSet) {
					MarlinSerial::store_char((uint8_t)(MarlinSerial::hlpuart1.Instance->RDR & 0x00FFU));
				} else {
					UNUSED(MarlinSerial::hlpuart1.Instance->RDR);
				}
				// Clear RXNE interrupt flag
				MarlinSerial::UartSendReq(&MarlinSerial::hlpuart1, MarlinSerial::UART_RXDATA_FLUSH_REQUEST);
			}
		}
		if (MarlinSerial::UartGetIT(&MarlinSerial::hlpuart1, UART_IT_TXE) != RESET) {
		    if (MarlinSerial::UartGetITSource(&MarlinSerial::hlpuart1, UART_IT_TXE) != RESET)
			{
		    	uint8_t const tail = MarlinSerial::tx_buffer.tail;
				if (MarlinSerial::tx_buffer.head == tail) {
					MarlinSerial::UartDisableIT(&MarlinSerial::hlpuart1, UART_IT_TXE);
					MarlinSerial::UartEnableIT(&MarlinSerial::hlpuart1, UART_IT_TC);
				} else {
					// If interrupts are enabled, there must be more data in the output
					// buffer. Send the next byte
					const uint8_t t = MarlinSerial::tx_buffer.tail;
					const uint8_t c = MarlinSerial::tx_buffer.buffer[t];
					MarlinSerial::tx_buffer.tail = (t + 1U) & static_cast<uint8_t>(TX_BUFFER_SIZE - 1U);

					MarlinSerial::hlpuart1.Instance->TDR = c;
				}
			}
		}
		if (MarlinSerial::UartGetIT(&MarlinSerial::hlpuart1, UART_IT_TC) != RESET) {
		    if (MarlinSerial::UartGetITSource(&MarlinSerial::hlpuart1, UART_IT_TC) != RESET)
			{
				MarlinSerial::UartDisableIT(&MarlinSerial::hlpuart1, UART_IT_TC);
			}
		}
	}

	//
	// @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
	//
	void USART2_IRQHandler(void)
	{
		if (MarlinSerial::UartGetIT(&MarlinSerial::huart2, UART_IT_RXNE) != RESET) {
			if (MarlinSerial::UartGetITSource(&MarlinSerial::huart2, UART_IT_RXNE) != RESET)
			{
				// read only of no errors, else skip byte
				bool hasErrorBitSet = false;
				if (MarlinSerial::UartGetIT(&MarlinSerial::huart2, UART_IT_NE) != RESET) {
					hasErrorBitSet = true;
				} else if (MarlinSerial::UartGetIT(&MarlinSerial::huart2, UART_IT_FE) != RESET) {
					hasErrorBitSet = true;
				} else if (MarlinSerial::UartGetIT(&MarlinSerial::huart2, UART_IT_PE) != RESET) {
					hasErrorBitSet = true;
				} else {
					hasErrorBitSet = false;
				}
				if (hasErrorBitSet) {
					MarlinSerial::store_char((uint8_t)(MarlinSerial::huart2.Instance->RDR & 0x00FFU));
				} else {
					UNUSED(MarlinSerial::huart2.Instance->RDR);
				}
				// Clear RXNE interrupt flag
				MarlinSerial::UartSendReq(&MarlinSerial::huart2, MarlinSerial::UART_RXDATA_FLUSH_REQUEST);
			}
		}
		if (MarlinSerial::UartGetIT(&MarlinSerial::huart2, UART_IT_TXE) != RESET) {
			if (MarlinSerial::UartGetITSource(&MarlinSerial::huart2, UART_IT_TXE) != RESET)
			{
				const uint8_t tail = MarlinSerial::tx_buffer.tail;
				if (MarlinSerial::tx_buffer.head == tail) {
					MarlinSerial::UartDisableIT(&MarlinSerial::huart2, UART_IT_TXE);
					MarlinSerial::UartEnableIT(&MarlinSerial::huart2, UART_IT_TC);
				} else {
					// If interrupts are enabled, there must be more data in the output
					// buffer. Send the next byte
					const uint8_t t = MarlinSerial::tx_buffer.tail;
					uint8_t const c = MarlinSerial::tx_buffer.buffer[t];
					MarlinSerial::tx_buffer.tail = (t + 1U) & static_cast<uint8_t>(TX_BUFFER_SIZE - 1U);

					MarlinSerial::huart2.Instance->TDR = c;
				}
			}
		}
		if (MarlinSerial::UartGetIT(&MarlinSerial::huart2, UART_IT_TC) != RESET) {
			if (MarlinSerial::UartGetITSource(&MarlinSerial::huart2, UART_IT_TC) != RESET)
			{
				MarlinSerial::UartDisableIT(&MarlinSerial::huart2, UART_IT_TC);
			}
		}
	}
}
