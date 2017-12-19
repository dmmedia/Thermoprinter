/*
 * HardwareSerial.cpp
 *
 *  Created on: 8. nov 2017
 *      Author: Den
 */

#include "main.h"
#include "gpio.h"
#include "macros.h"
#include <MarlinSerial.h>
#include "Configuration.h"
#include "SREGEmulation.h"
#include "rcc.h"

#if !defined(USBCON) && (defined(LPUART1) || defined(USART1) || defined(USART2))

UartHandle hlpuart1 { };
UartHandle huart2 { };

uint8_t rx_buf[RXBUF_LEN] { }, tx_buf[TXBUF_LEN] { };
/* xx_i - counter of input bytes (tx - pushed for transmit, rx - received)
   xx_o - counter of output bytes (tx - transmitted, rx - parsed)
   xx_e - counter of echoed bytes */
volatile uint16_t rx_i = 0, tx_o = 0;
uint16_t rx_o = 0, rx_e = 0, tx_i = 0;

#if UART_PRESENT(SERIAL_PORT)
  ring_buffer_r rx_buffer = { { 0 }, 0, 0 };
  #if TX_BUFFER_SIZE > 0
    ring_buffer_t tx_buffer = { { 0 }, 0, 0 };
    static bool _written { };
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
  int v { };
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

void UartMspInit(UartHandle* huart)
{
  GpioInit_t GPIO_InitStruct { };
  if(huart->Instance == LPUART1)
  {
    /* Peripheral clock enable */
	  RCC_LPUART1_CLK_ENABLE();

    /**LPUART1 GPIO Configuration
//    PA6     ------> LPUART1_CTS
    PC4     ------> LPUART1_TX
    PC5     ------> LPUART1_RX
//    PB1     ------> LPUART1_RTS
    */
//    GPIO_InitStruct.Pin = GPIO_PIN_6;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF4_LPUART1;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_LPUART1;
    GpioInit(GPIOC, &GPIO_InitStruct);

//    GPIO_InitStruct.Pin = GPIO_PIN_1;
//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//    GPIO_InitStruct.Alternate = GPIO_AF4_LPUART1;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* LPUART1 interrupt Init */
    NVIC_SetPriority(AES_RNG_LPUART1_IRQn, 0);
    NVIC_EnableIRQ(AES_RNG_LPUART1_IRQn);
  }
  else if(huart->Instance == USART2)
  {
    /* Peripheral clock enable */
    RCC_USART2_CLK_ENABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART2;
    GpioInit(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    NVIC_SetPriority(USART2_IRQn, 0);
    NVIC_EnableIRQ(USART2_IRQn);
  }

}

/**
  * @brief Configure the UART peripheral advanced features.
  * @param huart: UART handle.
  * @retval None
  */
void UART_AdvFeatureConfig(UartHandle *huart)
{
  /* if required, configure TX pin active level inversion */
  if(IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_TXINVERT_INIT))
  {
    MODIFY_REG(huart->Instance->CR2, USART_CR2_TXINV, huart->AdvancedInit.TxPinLevelInvert);
  }

  /* if required, configure RX pin active level inversion */
  if(IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_RXINVERT_INIT))
  {
    MODIFY_REG(huart->Instance->CR2, USART_CR2_RXINV, huart->AdvancedInit.RxPinLevelInvert);
  }

  /* if required, configure data inversion */
  if(IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_DATAINVERT_INIT))
  {
    MODIFY_REG(huart->Instance->CR2, USART_CR2_DATAINV, huart->AdvancedInit.DataInvert);
  }

  /* if required, configure RX/TX pins swap */
  if(IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_SWAP_INIT))
  {
    MODIFY_REG(huart->Instance->CR2, USART_CR2_SWAP, huart->AdvancedInit.Swap);
  }

  /* if required, configure RX overrun detection disabling */
  if(IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_RXOVERRUNDISABLE_INIT))
  {
    MODIFY_REG(huart->Instance->CR3, USART_CR3_OVRDIS, huart->AdvancedInit.OverrunDisable);
  }

  /* if required, configure DMA disabling on reception error */
  if(IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_DMADISABLEONERROR_INIT))
  {
    MODIFY_REG(huart->Instance->CR3, USART_CR3_DDRE, huart->AdvancedInit.DMADisableonRxError);
  }

  /* if required, configure auto Baud rate detection scheme */
  if(IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_AUTOBAUDRATE_INIT))
  {
    MODIFY_REG(huart->Instance->CR2, USART_CR2_ABREN, huart->AdvancedInit.AutoBaudRateEnable);
    /* set auto Baudrate detection parameters if detection is enabled */
    if(huart->AdvancedInit.AutoBaudRateEnable == UART_ADVFEATURE_AUTOBAUDRATE_ENABLE)
    {
      MODIFY_REG(huart->Instance->CR2, USART_CR2_ABRMODE, huart->AdvancedInit.AutoBaudRateMode);
    }
  }

  /* if required, configure MSB first on communication line */
  if(IS_BIT_SET(huart->AdvancedInit.AdvFeatureInit, UART_ADVFEATURE_MSBFIRST_INIT))
  {
    MODIFY_REG(huart->Instance->CR2, USART_CR2_MSBFIRST, huart->AdvancedInit.MSBFirst);
  }
}

/**
  * @brief  Returns the PCLK1 frequency
  * @note   Each time PCLK1 changes, this function must be called to update the
  *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK1 frequency
  */
uint32_t RCC_GetPCLK1Freq(void)
{
  /* Get HCLK source and Compute PCLK1 frequency ---------------------------*/
  return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_BITNUMBER]);
}

/**
  * @brief  Returns the PCLK2 frequency
  * @note   Each time PCLK2 changes, this function must be called to update the
  *         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK2 frequency
  */
uint32_t RCC_GetPCLK2Freq(void)
{
  /* Get HCLK source and Compute PCLK2 frequency ---------------------------*/
  return (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_BITNUMBER]);
}

/**
  * @brief  Return the peripheral clock frequency
  * @note   Return 0 if peripheral clock is unknown
  * @param  PeriphClk Peripheral clock identifier
  *         This parameter can be one of the following values:
  *            @arg @ref RCC_PERIPHCLK_RTC      RTC peripheral clock
  *            @arg @ref RCC_PERIPHCLK_LCD      LCD peripheral clock (*)
  *            @arg @ref RCC_PERIPHCLK_USB      USB or RNG peripheral clock (*)
  *            @arg @ref RCC_PERIPHCLK_USART1   USART1 peripheral clock (*)
  *            @arg @ref RCC_PERIPHCLK_USART2   USART2 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_LPUART1  LPUART1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2C1     I2C1 peripheral clock
  *            @arg @ref RCC_PERIPHCLK_I2C2     I2C2 peripheral clock (*)
  *            @arg @ref RCC_PERIPHCLK_I2C3     I2C3 peripheral clock (*)
  * @note   (*) means that this peripheral is not present on all the devices
  * @retval Frequency in Hz (0: means that no available frequency for the peripheral)
  */
uint32_t RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk)
{
  uint32_t frequency = 0U;
  uint32_t srcclk = 0U;
#if defined(USB)
    uint32_t pllmul = 0U, plldiv = 0U, pllvco = 0U;
#endif /* USB */

  switch (PeriphClk)
  {
  case RCC_PERIPHCLK_RTC:
#if defined(USB)
   case RCC_PERIPHCLK_USB:
    {
        /* Get the current USB source */
        srcclk = RCC_GET_USB_SOURCE();

        if((srcclk == RCC_USBCLKSOURCE_PLL) && (IS_BIT_SET(RCC->CR, RCC_CR_PLLRDY)))
        {
            /* Get PLL clock source and multiplication factor ----------------------*/
            pllmul = RCC->CFGR & RCC_CFGR_PLLMUL;
            plldiv = RCC->CFGR & RCC_CFGR_PLLDIV;
            pllmul = PLLMulTable[(pllmul >> RCC_CFGR_PLLMUL_Pos)];
            plldiv = (plldiv >> RCC_CFGR_PLLDIV_Pos) + 1U;

            /* Compute PLL clock input */
            if(RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSI)
            {
                if (READ_BIT(RCC->CR, RCC_CR_HSIDIVF) != 0U)
                {
                    pllvco =  (HSI_VALUE >> 2U);
                }
                else
                {
                    pllvco =  HSI_VALUE;
                }
            }
            else /* HSE source */
            {
                pllvco = HSE_VALUE;
            }
            /* pllvco * pllmul / plldiv */
            pllvco = (pllvco * pllmul);
            frequency = (pllvco/ plldiv);

        }
        else if((srcclk == RCC_USBCLKSOURCE_HSI48) && (IS_BIT_SET(RCC->CRRCR, RCC_CRRCR_HSI48RDY)))
        {
            frequency = HSI48_VALUE;
        }
        else /* RCC_USBCLKSOURCE_NONE */
        {
            frequency = 0U;
        }
        break;
    }
#endif /* USB */
#if defined(RCC_CCIPR_USART1SEL)
  case RCC_PERIPHCLK_USART1:
    {
      /* Get the current USART1 source */
      srcclk = RCC_GET_USART1_SOURCE();

      /* Check if USART1 clock selection is PCLK2 */
      if (srcclk == RCC_USART1CLKSOURCE_PCLK2)
      {
        frequency = RCC_GetPCLK2Freq();
      }
      /* Check if HSI is ready and if USART1 clock selection is HSI */
      else if ((srcclk == RCC_USART1CLKSOURCE_HSI) && (IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY)))
      {
        frequency = HSI_VALUE;
      }
      /* Check if USART1 clock selection is SYSCLK */
      else if (srcclk == RCC_USART1CLKSOURCE_SYSCLK)
      {
        frequency = RCC_GetSysClockFreq();
      }
      /* Check if LSE is ready  and if USART1 clock selection is LSE */
      else if ((srcclk == RCC_USART1CLKSOURCE_LSE) && (IS_BIT_SET(RCC->CSR, RCC_CSR_LSERDY)))
      {
        frequency = LSE_VALUE;
      }
      /* Clock not enabled for USART1*/
      else
      {
        frequency = 0U;
      }
      break;
    }
#endif /* RCC_CCIPR_USART1SEL */
  case RCC_PERIPHCLK_USART2:
    {
      /* Get the current USART2 source */
      srcclk = RCC_GET_USART2_SOURCE();

      /* Check if USART2 clock selection is PCLK1 */
      if (srcclk == RCC_USART2CLKSOURCE_PCLK1)
      {
        frequency = RCC_GetPCLK1Freq();
      }
      /* Check if HSI is ready and if USART2 clock selection is HSI */
      else if ((srcclk == RCC_USART2CLKSOURCE_HSI) && (IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY)))
      {
        frequency = HSI_VALUE;
      }
      /* Check if USART2 clock selection is SYSCLK */
      else if (srcclk == RCC_USART2CLKSOURCE_SYSCLK)
      {
        frequency = RCC_GetSysClockFreq();
      }
      /* Check if LSE is ready  and if USART2 clock selection is LSE */
      else if ((srcclk == RCC_USART2CLKSOURCE_LSE) && (IS_BIT_SET(RCC->CSR, RCC_CSR_LSERDY)))
      {
        frequency = LSE_VALUE;
      }
      /* Clock not enabled for USART2*/
      else
      {
        frequency = 0U;
      }
      break;
    }
  case RCC_PERIPHCLK_LPUART1:
    {
      /* Get the current LPUART1 source */
      srcclk = RCC_GET_LPUART1_SOURCE();

      /* Check if LPUART1 clock selection is PCLK1 */
      if (srcclk == RCC_LPUART1CLKSOURCE_PCLK1)
      {
        frequency = RCC_GetPCLK1Freq();
      }
      /* Check if HSI is ready and if LPUART1 clock selection is HSI */
      else if ((srcclk == RCC_LPUART1CLKSOURCE_HSI) && (IS_BIT_SET(RCC->CR, RCC_CR_HSIRDY)))
      {
        frequency = HSI_VALUE;
      }
      /* Check if LPUART1 clock selection is SYSCLK */
      else if (srcclk == RCC_LPUART1CLKSOURCE_SYSCLK)
      {
        frequency = RCC_GetSysClockFreq();
      }
      /* Check if LSE is ready  and if LPUART1 clock selection is LSE */
      else if ((srcclk == RCC_LPUART1CLKSOURCE_LSE) && (IS_BIT_SET(RCC->CSR, RCC_CSR_LSERDY)))
      {
        frequency = LSE_VALUE;
      }
      /* Clock not enabled for LPUART1*/
      else
      {
        frequency = 0U;
      }
      break;
    }
  default:
    {
      break;
    }
  }
  return(frequency);
}

/**
  * @brief Configure the UART peripheral.
  * @param huart: UART handle.
  * @retval HAL status
  */
StatusTypeDef UartSetConfig(UartHandle *huart)
{
  uint32_t tmpreg                     = 0x00000000U;
  UART_ClockSourceTypeDef clocksource = UART_CLOCKSOURCE_UNDEFINED;
  uint16_t brrtemp                    = 0x0000U;
  uint16_t usartdiv                   = 0x0000U;
  StatusTypeDef ret               = STATUS_OK;

  /*-------------------------- USART CR1 Configuration -----------------------*/
  /* Clear M, PCE, PS, TE, RE and OVER8 bits and configure
   *  the UART Word Length, Parity, Mode and oversampling:
   *  set the M bits according to huart->Init.WordLength value
   *  set PCE and PS bits according to huart->Init.Parity value
   *  set TE and RE bits according to huart->Init.Mode value
   *  set OVER8 bit according to huart->Init.OverSampling value */
  tmpreg = (uint32_t)huart->Init.WordLength | huart->Init.Parity | huart->Init.Mode | huart->Init.OverSampling ;
  MODIFY_REG(huart->Instance->CR1, UART_CR1_FIELDS, tmpreg);

  /*-------------------------- USART CR2 Configuration -----------------------*/
  /* Configure the UART Stop Bits: Set STOP[13:12] bits according
   * to huart->Init.StopBits value */
  MODIFY_REG(huart->Instance->CR2, USART_CR2_STOP, huart->Init.StopBits);

  /*-------------------------- USART CR3 Configuration -----------------------*/
  /* Configure
   * - UART HardWare Flow Control: set CTSE and RTSE bits according
   *   to huart->Init.HwFlowCtl value
   * - one-bit sampling method versus three samples' majority rule according
   *   to huart->Init.OneBitSampling (not applicable to LPUART) */
  tmpreg = (uint32_t)huart->Init.HwFlowCtl;
  if (!(UART_INSTANCE_LOWPOWER(huart)))
  {
    tmpreg |= huart->Init.OneBitSampling;
  }
  MODIFY_REG(huart->Instance->CR3, (USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_ONEBIT), tmpreg);

  /*-------------------------- USART BRR Configuration -----------------------*/
  UART_GETCLOCKSOURCE(huart, clocksource);
  uint32_t frequency = RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_USART2);

  /* Check LPUART instance */
  if(UART_INSTANCE_LOWPOWER(huart))
  {
    /* Retrieve frequency clock */
    tmpreg = 0;

    switch (clocksource)
    {
    case UART_CLOCKSOURCE_PCLK1:
      tmpreg = RCC_GetPCLK1Freq();
      break;
    case UART_CLOCKSOURCE_HSI:
      if (RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
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
    default:
        ret = STATUS_ERROR;
      break;
    }

    /* if proper clock source reported */
    if (tmpreg != 0)
    {
      /* ensure that Frequency clock is in the range [3 * baudrate, 4096 * baudrate] */
      if ( (tmpreg < (3 * huart->Init.BaudRate) ) ||
           (tmpreg > (4096 * huart->Init.BaudRate) ))
      {
        ret = STATUS_ERROR;
      }
      else
      {
        tmpreg = (uint32_t)(UART_DIV_LPUART(tmpreg, huart->Init.BaudRate));

        if ((tmpreg >= UART_LPUART_BRR_MIN) && (tmpreg <= UART_LPUART_BRR_MAX))
        {
           huart->Instance->BRR = tmpreg;
        }
        else
        {
          ret = STATUS_ERROR;
        }
      }  /*   if ( (tmpreg < (3 * huart->Init.BaudRate) ) || (tmpreg > (4096 * huart->Init.BaudRate) )) */
    } /* if (tmpreg != 0) */
  }
  /* Check UART Over Sampling to set Baud Rate Register */
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
      if (RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
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
      huart->Instance->BRR = (uint16_t)(UART_DIV_SAMPLING16(RCC_GetPCLK1Freq(), huart->Init.BaudRate));
      break;
    case UART_CLOCKSOURCE_PCLK2:
      huart->Instance->BRR = (uint16_t)(UART_DIV_SAMPLING16(RCC_GetPCLK2Freq(), huart->Init.BaudRate));
      break;
    case UART_CLOCKSOURCE_HSI:
      if (RCC_GET_FLAG(RCC_FLAG_HSIDIV) != 0U)
      {
        huart->Instance->BRR = (uint16_t)(UART_DIV_SAMPLING16((HSI_VALUE >> 2U), huart->Init.BaudRate));
      }
      else
      {
        huart->Instance->BRR = (uint16_t)(UART_DIV_SAMPLING16(HSI_VALUE, huart->Init.BaudRate));
      }
      break;
    case UART_CLOCKSOURCE_SYSCLK:
      huart->Instance->BRR = (uint16_t)(UART_DIV_SAMPLING16(RCC_GetSysClockFreq(), huart->Init.BaudRate));
      break;
    case UART_CLOCKSOURCE_LSE:
      huart->Instance->BRR = (uint16_t)(UART_DIV_SAMPLING16(LSE_VALUE, huart->Init.BaudRate));
      break;
    case UART_CLOCKSOURCE_UNDEFINED:
    default:
        ret = STATUS_ERROR;
      break;
    }
  }

  return ret;

}

/**
  * @brief  Handle UART Communication Timeout.
  * @param  huart UART handle.
  * @param  Flag Specifies the UART flag to check
  * @param  Status Flag status (SET or RESET)
  * @param  Tickstart Tick start value
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
StatusTypeDef UART_WaitOnFlagUntilTimeout(UartHandle *huart, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout)
{
  /* Wait until flag is set */
  while((UART_GET_FLAG(huart, Flag) ? SET : RESET) == Status)
  {
    /* Check for the Timeout */
    if(Timeout != UART_MAX_DELAY)
    {
      if((Timeout == 0) || ((GetTick()-Tickstart) > Timeout))
      {
        /* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts for the interrupt process */
        CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE | USART_CR1_TXEIE));
        CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

        huart->gState  = UART_STATE_READY;
        huart->RxState = UART_STATE_READY;

        /* Process Unlocked */
        UNLOCK_HANDLE(huart);
        return STATUS_TIMEOUT;
      }
    }
  }
  return STATUS_OK;
}

/**
  * @brief Check the UART Idle State.
  * @param huart UART handle.
  * @retval HAL status
  */
StatusTypeDef UART_CheckIdleState(UartHandle *huart)
{
  uint32_t tickstart = 0;

  /* Initialize the UART ErrorCode */
  huart->ErrorCode = UART_ERROR_NONE;

  /* Init tickstart for timeout managment*/
  tickstart = GetTick();

  /* Check if the Transmitter is enabled */
  if((huart->Instance->CR1 & USART_CR1_TE) == USART_CR1_TE)
  {
    /* Wait until TEACK flag is set */
    if(UART_WaitOnFlagUntilTimeout(huart, USART_ISR_TEACK, RESET, tickstart, HAL_UART_TIMEOUT_VALUE) != STATUS_OK)
    {
      /* Timeout occurred */
      return STATUS_TIMEOUT;
    }
  }
  /* Check if the Receiver is enabled */
  if((huart->Instance->CR1 & USART_CR1_RE) == USART_CR1_RE)
  {
    /* Wait until REACK flag is set */
    if(UART_WaitOnFlagUntilTimeout(huart, USART_ISR_REACK, RESET, tickstart, HAL_UART_TIMEOUT_VALUE) != STATUS_OK)
    {
      /* Timeout occurred */
      return STATUS_TIMEOUT;
    }
  }

  /* Initialize the UART State */
  huart->gState  = UART_STATE_READY;
  huart->RxState = UART_STATE_READY;

  /* Process Unlocked */
  UNLOCK_HANDLE(huart);

  return STATUS_OK;
}

/**
  * @brief Initialize the UART mode according to the specified
  *        parameters in the UART_InitTypeDef and initialize the associated handle.
  * @param huart: UART handle.
  * @retval HAL status
  */
StatusTypeDef UartInit(UartHandle *huart)
{
  /* Check the UART handle allocation */
  if(huart == NULL)
  {
    return STATUS_ERROR;
  }

  if(huart->gState == UART_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    huart->Lock = HANDLE_UNLOCKED;

    /* Init the low level hardware : GPIO, CLOCK */
    UartMspInit(huart);
  }

  huart->gState = UART_STATE_BUSY;

  /* Disable the Peripheral */
  UART_DISABLE(huart);

  /* Set the UART Communication parameters */
  if (huart->AdvancedInit.AdvFeatureInit != UART_ADVFEATURE_NO_INIT)
  {
    UART_AdvFeatureConfig(huart);
  }

  if (UartSetConfig(huart) == STATUS_ERROR)
  {
    return STATUS_ERROR;
  }

  /* In asynchronous mode, the following bits must be kept cleared:
  - LINEN and CLKEN bits in the USART_CR2 register,
  - SCEN, HDSEL and IREN  bits in the USART_CR3 register.*/
  CLEAR_BIT(huart->Instance->CR2, (USART_CR2_LINEN | USART_CR2_CLKEN));
  CLEAR_BIT(huart->Instance->CR3, (USART_CR3_SCEN | USART_CR3_HDSEL | USART_CR3_IREN));

  /* Enable the Peripheral */
  UART_ENABLE(huart);

  /* TEACK and/or REACK to check before moving huart->gState and huart->RxState to Ready */
  return (UART_CheckIdleState(huart));
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
	if (UartInit(&hlpuart1) != STATUS_OK)
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
    if (UartInit(&huart2) != STATUS_OK)
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

void UartMspDeInit(UartHandle* huart)
{

  if(huart->Instance == LPUART1)
  {
    /* Peripheral clock disable */
    RCC_LPUART1_CLK_DISABLE();

    /**LPUART1 GPIO Configuration
//    PA6     ------> LPUART1_CTS
    PC4     ------> LPUART1_TX
    PC5     ------> LPUART1_RX
//    PB1     ------> LPUART1_RTS
    */
//    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);

    GpioDeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

//    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

    /* LPUART1 interrupt DeInit */
    NVIC_DisableIRQ(AES_RNG_LPUART1_IRQn);
  }
  else if(huart->Instance == USART2)
  {
    /* Peripheral clock disable */
    RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GpioDeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt DeInit */
    NVIC_DisableIRQ(USART2_IRQn);
  }
}

/**
  * @brief DeInitialize the UART peripheral.
  * @param huart: UART handle.
  * @retval HAL status
  */
StatusTypeDef UartDeInit(UartHandle *huart)
{
  /* Check the UART handle allocation */
  if(huart == NULL)
  {
    return STATUS_ERROR;
  }

  huart->gState = UART_STATE_BUSY;

  /* Disable the Peripheral */
  UART_DISABLE(huart);

  huart->Instance->CR1 = 0x0U;
  huart->Instance->CR2 = 0x0U;
  huart->Instance->CR3 = 0x0U;

  /* DeInit the low level hardware */
  UartMspDeInit(huart);

  huart->ErrorCode = UART_ERROR_NONE;
  huart->gState    = UART_STATE_RESET;
  huart->RxState   = UART_STATE_RESET;

  /* Process Unlock */
  UNLOCK_HANDLE(huart);

  return STATUS_OK;
}

void MarlinSerial::end() {
	UartDeInit(&huart2);
//	UartDeInit(&hlpuart1);
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
        UartDisableIT(&hlpuart1, UART_IT_TC);
    }

    /* And never call default handler */
    return;
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
        UartDisableIT(&huart2, UART_IT_TC);
    }
    /* And never call default handler */
    return;
}

void transmit(UartHandle *huart, const uint8_t byte)
{
  CRITICAL_SECTION_START;
  tx_buf[TXBUF_MSK & tx_i] = byte;
  tx_i++;
  CRITICAL_SECTION_END;
  UartEnableIT(huart, UART_IT_TXE);
}

void UartDisableIT(UartHandle *huart, uint32_t source) {
  (
	((((uint8_t)source) >> 5U) == 1U) ?
	  (huart->Instance->CR1 &= ~(1U << (source & UART_IT_MASK))) :
	  ((((uint8_t)source) >> 5U) == 2U) ?
		(huart->Instance->CR2 &= ~(1U << (source & UART_IT_MASK))) :
        (huart->Instance->CR3 &= ~(1U << (source & UART_IT_MASK)))
  );
}

void UartEnableIT(UartHandle *huart, uint32_t source) {
  (((((uint8_t)source) >> 5U) == 1U) ?
    (huart->Instance->CR1 |= (1U << (source & UART_IT_MASK))) :
    ((((uint8_t)source) >> 5U) == 2U) ?
	  (huart->Instance->CR2 |= (1U << (source & UART_IT_MASK))) :
      (huart->Instance->CR3 |= (1U << (source & UART_IT_MASK)))
  );
}

ITStatus UartGetITSource(UartHandle *huart, uint32_t source) {
  return ((((((uint8_t)source) >> 5U) == 1U) ?
	huart->Instance->CR1 :
	(((((uint8_t)source) >> 5U) == 2U) ?
	  huart->Instance->CR2 :
	    huart->Instance->CR3
	)
   ) & ((uint32_t)1U << (((uint16_t)source) & UART_IT_MASK))
  ) > 0 ? SET : RESET;
}

ITStatus UartGetIT(UartHandle *huart, uint32_t source) {
  return (huart->Instance->ISR & ((uint32_t)1U << (source >> 0x08U))) > 0 ? SET : RESET;
}

void UartSendReq(UartHandle *huart, uint32_t req) {
  (huart->Instance->RQR |= (uint32_t)req);
}

void MarlinSerial::write(const uint8_t c) {
  transmit(&huart2, c);
}

// Preinstantiate
MarlinSerial customizedSerial { };

#endif // !USBCON && (UBRRH || UBRR0H || UBRR1H || UBRR2H || UBRR3H)

// For USB targets use the UART for BT interfacing
#if ENABLED(BLUETOOTH)
  MarlinSerial bluetoothSerial;
#endif
