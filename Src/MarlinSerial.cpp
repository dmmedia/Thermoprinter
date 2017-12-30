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
#include "SREGEmulation.h"
#include "gpio.h"
#include "Configuration.h"
#include "rcc.h"
#include <MarlinSerial.h>

namespace MarlinSerial {

	UartHandle hlpuart1;
	UartHandle huart2;

	uint8_t rx_buf[RXBUF_LEN];
	uint8_t tx_buf[TXBUF_LEN];
	// xx_i - counter of input bytes (tx - pushed for transmit, rx - received)
	// xx_o - counter of output bytes (tx - transmitted, rx - parsed)
	// xx_e - counter of echoed bytes
	volatile uint16_t rx_i = 0U;
	volatile uint16_t tx_o = 0U;
	uint16_t rx_o = 0U;
	uint16_t rx_e = 0U;
	uint16_t tx_i = 0U;

	ring_buffer_r rx_buffer = { { 0U }, 0U, 0U };
	#if TX_BUFFER_SIZE > 0U
		ring_buffer_t tx_buffer = { { 0U }, 0U, 0U };
		bool written;
	#endif

	void flush(void) {
		// RX
		// don't reverse this or there may be problems if the RX interrupt
		// occurs after reading the value of rx_buffer_head but before writing
		// the value to rx_buffer_tail; the previous value of rx_buffer_head
		// may be written to rx_buffer_tail, making it appear as if the buffer
		// were full, not empty.
		noInterrupts();
			rx_buffer.head = rx_buffer.tail;
		interrupts();
	}

	uint8_t available(void) {
		noInterrupts();
		const uint8_t h = rx_buffer.head;
		const uint8_t t = rx_buffer.tail;
		interrupts();
		return ((RX_BUFFER_SIZE + h) - t) & static_cast<uint8_t>(RX_BUFFER_SIZE - 1U);
	}

	int32_t read(void) {
		int32_t v;
		noInterrupts();
		const uint8_t t = rx_buffer.tail;
		if (rx_buffer.head == t) {
			v = -1;
		} else {
			v = static_cast<int32_t>(rx_buffer.buffer[t]);
			rx_buffer.tail = (t + 1U) & static_cast<uint8_t>(RX_BUFFER_SIZE - 1U);
		}
		interrupts();
		return v;
	}

	void UartMspInit(const UartHandle* const huart)
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
	void UART_AdvFeatureConfig(const UartHandle * const huart)
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
	StatusTypeDef UartSetConfig(UartHandle * const huart)
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
		const uint32_t frequency = RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_USART2);

		// Check LPUART instance
		if(UART_INSTANCE_LOWPOWER(huart) != RESET)
		{
			// Retrieve frequency clock
			tmpreg = 0;

			switch (clocksource)
			{
				case UART_CLOCKSOURCE_PCLK1:
					tmpreg = RCC_GetPCLK1Freq();
					break;
				case UART_CLOCKSOURCE_HSI:
					if (RCC_GetFlag(RCC_FLAG_HSIDIV) != 0U)
					{
						tmpreg = (uint32_t) (HSI_VALUE >> 2U);
					}
					else
					{
						tmpreg = (uint32_t) HSI_VALUE;
					}
					break;
				case UART_CLOCKSOURCE_SYSCLK:
					tmpreg = RCC_GetSysClockFreq();
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
					usartdiv = (uint16_t)(UART_DIV_SAMPLING8(RCC_GetPCLK2Freq(), huart->Init.BaudRate));
					break;
				case UART_CLOCKSOURCE_HSI:
					if (RCC_GetFlag(RCC_FLAG_HSIDIV) != 0U)
					{
						usartdiv = (uint16_t)(UART_DIV_SAMPLING8((HSI_VALUE >> 2U), huart->Init.BaudRate));
					}
					else
					{
						usartdiv = (uint16_t)(UART_DIV_SAMPLING8(HSI_VALUE, huart->Init.BaudRate));
					}
					break;
				case UART_CLOCKSOURCE_SYSCLK:
					usartdiv = (uint16_t)(UART_DIV_SAMPLING8(RCC_GetSysClockFreq(), huart->Init.BaudRate));
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
					huart->Instance->BRR = UART_DivSampling16(RCC_GetPCLK1Freq(), huart->Init.BaudRate);
					break;
				case UART_CLOCKSOURCE_PCLK2:
					huart->Instance->BRR = UART_DivSampling16(RCC_GetPCLK2Freq(), huart->Init.BaudRate);
					break;
				case UART_CLOCKSOURCE_HSI:
					if (RCC_GetFlag(RCC_FLAG_HSIDIV) != 0U)
					{
						huart->Instance->BRR = UART_DivSampling16((HSI_VALUE >> 2U), huart->Init.BaudRate);
					}
					else
					{
						huart->Instance->BRR = UART_DivSampling16(HSI_VALUE, huart->Init.BaudRate);
					}
					break;
				case UART_CLOCKSOURCE_SYSCLK:
					huart->Instance->BRR = UART_DivSampling16(RCC_GetSysClockFreq(), huart->Init.BaudRate);
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
	StatusTypeDef UART_WaitOnFlagUntilTimeout(UartHandle * const huart, uint32_t const Flag, FlagStatus const Status, uint32_t const Tickstart, uint32_t const Timeout)
	{
		StatusTypeDef res = STATUS_OK;
		// Wait until flag is set
		while(Status == UART_GetFlagStatus(huart, Flag))
		{
			// Check for the Timeout
			if(Timeout != UART_MAX_DELAY)
			{
				uint32_t const elapsedTime = GetTick() - Tickstart;
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
	///
	StatusTypeDef UART_CheckIdleState(UartHandle * const huart)
	{
		StatusTypeDef res = STATUS_OK;

		// Initialize the UART ErrorCode
		huart->ErrorCode = UART_ERROR_NONE;

		// Init tickstart for timeout managment
		uint32_t const tickstart = GetTick();

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
	StatusTypeDef UartInit(UartHandle * const huart)
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
	void MX_LPUART1_UART_Init(const uint32_t baud)
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
	void MX_USART2_UART_Init(const uint32_t baud)
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
			written = false;
	  	#endif
	}

	void UartMspDeInit(const UartHandle* const huart)
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
	///
	StatusTypeDef UartDeInit(UartHandle * const huart)
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

	void transmit(const UartHandle * const huart, const uint8_t byte)
	{
		noInterrupts();
		tx_buf[TXBUF_MSK & tx_i] = byte;
		tx_i++;
		interrupts();
		UartEnableIT(huart, UART_IT_TXE);
	}

	void UartDisableIT(const UartHandle * const huart, const uint32_t source) {
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

	void UartEnableIT(const UartHandle * const huart, const uint32_t source) {
		uint32_t const bitToSet = (uint32_t)1U << (source & UART_IT_MASK);
		if (((source & 0xFFU) >> 5U) == 1U) {
			SET_BIT(huart->Instance->CR1, bitToSet);
		} else if (((source & 0xFFU) >> 5U) == 2U) {
			SET_BIT(huart->Instance->CR2, bitToSet);
		} else {
			SET_BIT(huart->Instance->CR3, bitToSet);
		}
	}

	ITStatus UartGetITSource(const UartHandle * const huart, uint32_t const source) {
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

	ITStatus UartGetIT(const UartHandle * const huart, uint32_t const source) {
		ITStatus res = RESET;
		if ((huart->Instance->ISR & ((uint32_t)1U << (source >> 0x08U))) > 0U) {
			res = SET;
		}
		return  res;
	}

	void UartSendReq(const UartHandle * const huart, uint32_t const req) {
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

	void printNumber(uint32_t n, uint8_t const base) {
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
		print(static_cast<uint32_t>(n), base);
	}

	void print(char const c, int32_t const base) {
		int16_t const e = static_cast<int16_t>(c);
		uint8_t const f = static_cast<uint8_t>(static_cast<uint16_t>(e) & 0x00FFU);
		print(f, base);
	}


	void print(uint8_t const b, int32_t const base) {
		print(static_cast<uint32_t>(b), base);
	}

}

extern "C" {
	//
	// @brief This function handles AES, RNG and LPUART1 interrupts / LPUART1 wake-up interrupt through EXTI line 28.
	//
	void AES_RNG_LPUART1_IRQHandler(void)
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
