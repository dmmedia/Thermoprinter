/*
 * rcc.cpp
 *
 *  Created on: 17. dets 2017
 *      Author: Den
 */

#include <stm32l0xx.h>
#include "typedefs.h"
#include "macros.h"
#include "rcc.h"

/**
 * @brief uwTick_variable uwTick variable
 */
__IO uint32_t uwTick { };

/**
* @brief This function configures the source of the time base.
*        The time source is configured  to have 1ms time base with a dedicated
*        Tick interrupt priority.
* @note This function is called  automatically at the beginning of program after
*       reset by Init() or at any time when clock is reconfigured  by RCC_ClockConfig().
* @note In the default implementation, SysTick timer is the source of time base.
*       It is used to generate interrupts at regular time intervals.
*       Care must be taken if HAL_Delay() is called from a peripheral ISR process,
*       The the SysTick interrupt must have higher priority (numerically lower)
*       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
*       The function is declared as __Weak  to be overwritten  in case of other
*       implementation  in user file.
* @param TickPriority: Tick interrupt priority.
* @retval status
*/
StatusTypeDef InitTick(uint32_t TickPriority)
{
	/*Configure the SysTick to have interrupt in 1ms time basis*/
	SysTick_Config(SystemCoreClock/1000U);

	/*Configure the SysTick IRQ priority */
	NVIC_SetPriority(SysTick_IRQn, TickPriority);

	 /* Return function status */
	return STATUS_OK;
}

/**
 * @brief Provides a tick value in millisecond.
 * @retval tick value
 */
uint32_t GetTick(void)
{
    return uwTick;
}

/**
  * @brief This function is called to increment  a global variable "uwTick"
  *        used as application time base.
  * @note In the default implementation, this variable is incremented each 1ms
  *       in Systick ISR.
  * @retval None
  */
void IncTick(void)
{
  uwTick++;
}

/**
* @brief  Returns the SYSCLK frequency
* @note   The system frequency computed by this function is not the real
*         frequency in the chip. It is calculated based on the predefined
*         constant and the selected clock source:
* @note     If SYSCLK source is MSI, function returns a value based on MSI
*             Value as defined by the MSI range.
* @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
* @note     If SYSCLK source is HSE, function returns a value based on HSE_VALUE(**)
* @note     If SYSCLK source is PLL, function returns a value based on HSE_VALUE(**)
*           or HSI_VALUE(*) multiplied/divided by the PLL factors.
* @note     (*) HSI_VALUE is a constant defined in stm32l0xx_hal_conf.h file (default value
*               16 MHz) but the real value may vary depending on the variations
*               in voltage and temperature.
* @note     (**) HSE_VALUE is a constant defined in stm32l0xx_hal_conf.h file (default value
*                8 MHz), user has to ensure that HSE_VALUE is same as the real
*                frequency of the crystal used. Otherwise, this function may
*                have wrong result.
*
* @note   The result of this function could be not correct when using fractional
*         value for HSE crystal.
*
* @note   This function can be used by the user application to compute the
*         baud-rate for the communication peripherals or configure other parameters.
*
* @note   Each time SYSCLK changes, this function must be called to update the
*         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
*
* @retval SYSCLK frequency
*/
uint32_t RCC_GetSysClockFreq(void)
{
	uint32_t tmpreg = 0, pllm = 0, plld = 0, pllvco = 0, msiclkrange = 0;
	uint32_t sysclockfreq = 0;

	tmpreg = RCC->CFGR;

	/* Get SYSCLK source -------------------------------------------------------*/
	switch (tmpreg & RCC_CFGR_SWS)
	{
	  case RCC_SYSCLKSOURCE_STATUS_HSI:  /* HSI used as system clock source */
	  {
		if ((RCC->CR & RCC_CR_HSIDIVF) != 0)
		{
		  sysclockfreq =  (HSI_VALUE >> 2);
		}
		else
		{
		  sysclockfreq =  HSI_VALUE;
		}
		break;
	  }
	  case RCC_SYSCLKSOURCE_STATUS_HSE:  /* HSE used as system clock */
	  {
		sysclockfreq = HSE_VALUE;
		break;
	  }
	  case RCC_SYSCLKSOURCE_STATUS_PLLCLK:  /* PLL used as system clock */
	  {
		pllm = PLLMulTable[(uint32_t)(tmpreg & RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_BITNUMBER];
		plld = ((uint32_t)(tmpreg & RCC_CFGR_PLLDIV) >> RCC_CFGR_PLLDIV_BITNUMBER) + 1;
		if (RCC_GET_PLL_OSCSOURCE() != RCC_PLLSOURCE_HSI)
		{
		  /* HSE used as PLL clock source */
		  pllvco = (HSE_VALUE * pllm) / plld;
		}
		else
		{
		  if ((RCC->CR & RCC_CR_HSIDIVF) != 0)
		  {
			pllvco = ((HSI_VALUE >> 2) * pllm) / plld;
		  }
		  else
		  {
		   pllvco = (HSI_VALUE * pllm) / plld;
		  }
		}
		sysclockfreq = pllvco;
		break;
	  }
	  case RCC_SYSCLKSOURCE_STATUS_MSI:  /* MSI used as system clock source */
	  default: /* MSI used as system clock */
	  {
		msiclkrange = (RCC->ICSCR & RCC_ICSCR_MSIRANGE ) >> RCC_ICSCR_MSIRANGE_BITNUMBER;
		sysclockfreq = (32768 * (1 << (msiclkrange + 1)));
		break;
	  }
	}
	return sysclockfreq;
}

/**
  * @brief  Initializes the RCC Oscillators according to the specified parameters in the
  *         RCC_OscInitTypeDef.
  * @param  RCC_OscInitStruct pointer to an RCC_OscInitTypeDef structure that
  *         contains the configuration information for the RCC Oscillators.
  * @note   The PLL is not disabled when used as system clock.
  * @note   Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not
  *         supported by this macro. User should request a transition to LSE Off
  *         first and then LSE On or LSE Bypass.
  * @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
  *         supported by this macro. User should request a transition to HSE Off
  *         first and then HSE On or HSE Bypass.
  * @retval status
  */
StatusTypeDef RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
{
   uint32_t tickstart = 0U;

  /*------------------------------- HSE Configuration ------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) == RCC_OSCILLATORTYPE_HSE)
  {
    /* When the HSE is used as system clock or clock source for PLL in these cases it is not allowed to be disabled */
    if((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSE)
       || ((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && (RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE)))
    {
      if((RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET) && (RCC_OscInitStruct->HSEState == RCC_HSE_OFF))
      {
        return STATUS_ERROR;
      }
    }
    else
    {
      /* Set the new HSE configuration ---------------------------------------*/
      RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);


       /* Check the HSE State */
      if(RCC_OscInitStruct->HSEState != RCC_HSE_OFF)
      {
        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till HSE is ready */
        while(RCC_GET_FLAG(RCC_FLAG_HSERDY) == RESET)
        {
          if((GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }
      }
      else
      {
        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till HSE is disabled */
        while(RCC_GET_FLAG(RCC_FLAG_HSERDY) != RESET)
        {
           if((GetTick() - tickstart ) > HSE_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }
      }
    }
  }
  /*----------------------------- HSI Configuration --------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI)
  {
    /* Check if HSI is used as system clock or as PLL source when PLL is selected as system clock */
    if((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSI)
       || ((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK) && (RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSI)))
    {
      /* When HSI is used as system clock it will not disabled */
      if((RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET) && (RCC_OscInitStruct->HSIState != RCC_HSI_ON))
      {
        return STATUS_ERROR;
      }
      /* Otherwise, just the calibration is allowed */
      else
      {
        /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
        RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
      }
    }
    else
    {
      /* Check the HSI State */
      if(RCC_OscInitStruct->HSIState != RCC_HSI_OFF)
      {
        /* Enable the Internal High Speed oscillator (HSI or HSIdiv4) */
        RCC_HSI_CONFIG(RCC_OscInitStruct->HSIState);

        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till HSI is ready */
        while(RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
        {
          if((GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }

        /* Adjusts the Internal High Speed oscillator (HSI) calibration value.*/
        RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
      }
      else
      {
        /* Disable the Internal High Speed oscillator (HSI). */
        RCC_HSI_DISABLE();

        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till HSI is disabled */
        while(RCC_GET_FLAG(RCC_FLAG_HSIRDY) != RESET)
        {
          if((GetTick() - tickstart ) > HSI_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }
      }
    }
  }
  /*----------------------------- MSI Configuration --------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_MSI) == RCC_OSCILLATORTYPE_MSI)
  {
    /* When the MSI is used as system clock it will not be disabled */
    if((RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_MSI) )
    {
      if((RCC_GET_FLAG(RCC_FLAG_MSIRDY) != RESET) && (RCC_OscInitStruct->MSIState == RCC_MSI_OFF))
      {
        return STATUS_ERROR;
      }
      /* Otherwise, just the calibration and MSI range change are allowed */
      else
      {
        /* To correctly read data from FLASH memory, the number of wait states (LATENCY)
           must be correctly programmed according to the frequency of the CPU clock
           (HCLK) and the supply voltage of the device. */
        if(RCC_OscInitStruct->MSIClockRange > RCC_GET_MSI_RANGE())
        {
          /* First increase number of wait states update if necessary */
          if(RCC_SetFlashLatencyFromMSIRange(RCC_OscInitStruct->MSIClockRange) != STATUS_OK)
          {
            return STATUS_ERROR;
          }

          /* Selects the Multiple Speed oscillator (MSI) clock range .*/
          RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);
          /* Adjusts the Multiple Speed oscillator (MSI) calibration value.*/
          RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);
        }
        else
        {
          /* Else, keep current flash latency while decreasing applies */
          /* Selects the Multiple Speed oscillator (MSI) clock range .*/
          RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);
          /* Adjusts the Multiple Speed oscillator (MSI) calibration value.*/
          RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);

          /* Decrease number of wait states update if necessary */
          if(RCC_SetFlashLatencyFromMSIRange(RCC_OscInitStruct->MSIClockRange) != STATUS_OK)
          {
            return STATUS_ERROR;
          }
        }

        /* Update the SystemCoreClock global variable */
        SystemCoreClock =  (32768U * (1U << ((RCC_OscInitStruct->MSIClockRange >> RCC_ICSCR_MSIRANGE_BITNUMBER) + 1U)))
                           >> AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_BITNUMBER)];

        /* Configure the source of time base considering new system clocks settings*/
        InitTick (TICK_INT_PRIORITY);
      }
    }
    else
    {
      /* Check the MSI State */
      if(RCC_OscInitStruct->MSIState != RCC_MSI_OFF)
      {
        /* Enable the Multi Speed oscillator (MSI). */
        RCC_MSI_ENABLE();

        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till MSI is ready */
        while(RCC_GET_FLAG(RCC_FLAG_MSIRDY) == RESET)
        {
          if((GetTick() - tickstart) > MSI_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }
        /* Selects the Multiple Speed oscillator (MSI) clock range .*/
        RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);
         /* Adjusts the Multiple Speed oscillator (MSI) calibration value.*/
        RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);

      }
      else
      {
        /* Disable the Multi Speed oscillator (MSI). */
        RCC_MSI_DISABLE();

        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till MSI is ready */
        while(RCC_GET_FLAG(RCC_FLAG_MSIRDY) != RESET)
        {
          if((GetTick() - tickstart) > MSI_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }
      }
    }
  }
  /*------------------------------ LSI Configuration -------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI)
  {
    /* Check the LSI State */
    if(RCC_OscInitStruct->LSIState != RCC_LSI_OFF)
    {
      /* Enable the Internal Low Speed oscillator (LSI). */
      RCC_LSI_ENABLE();

      /* Get Start Tick */
      tickstart = GetTick();

      /* Wait till LSI is ready */
      while(RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET)
      {
        if((GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
        {
          return STATUS_TIMEOUT;
        }
      }
    }
    else
    {
      /* Disable the Internal Low Speed oscillator (LSI). */
      RCC_LSI_DISABLE();

      /* Get Start Tick */
      tickstart = GetTick();

      /* Wait till LSI is disabled */
      while(RCC_GET_FLAG(RCC_FLAG_LSIRDY) != RESET)
      {
        if((GetTick() - tickstart ) > LSI_TIMEOUT_VALUE)
        {
          return STATUS_TIMEOUT;
        }
      }
    }
  }
  /*------------------------------ LSE Configuration -------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE)
  {
    FlagStatus       pwrclkchanged = RESET;

    /* Update LSE configuration in Backup Domain control register    */
    /* Requires to enable write access to Backup Domain of necessary */
    if(RCC_PWR_IS_CLK_DISABLED())
    {
      RCC_PWR_CLK_ENABLE();
      pwrclkchanged = SET;
    }

    if(IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
    {
      /* Enable write access to Backup domain */
      SET_BIT(PWR->CR, PWR_CR_DBP);

      /* Wait for Backup domain Write protection disable */
      tickstart = GetTick();

      while(IS_BIT_CLR(PWR->CR, PWR_CR_DBP))
      {
        if((GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE)
        {
          return STATUS_TIMEOUT;
        }
      }
    }

    /* Set the new LSE configuration -----------------------------------------*/
    RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);
    /* Check the LSE State */
    if(RCC_OscInitStruct->LSEState != RCC_LSE_OFF)
    {
      /* Get Start Tick */
      tickstart = GetTick();

      /* Wait till LSE is ready */
      while(RCC_GET_FLAG(RCC_FLAG_LSERDY) == RESET)
      {
        if((GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
        {
          return STATUS_TIMEOUT;
        }
      }
    }
    else
    {
      /* Get Start Tick */
      tickstart = GetTick();

      /* Wait till LSE is disabled */
      while(RCC_GET_FLAG(RCC_FLAG_LSERDY) != RESET)
      {
        if((GetTick() - tickstart ) > RCC_LSE_TIMEOUT_VALUE)
        {
          return STATUS_TIMEOUT;
        }
      }
    }

    /* Require to disable power clock if necessary */
    if(pwrclkchanged == SET)
    {
      RCC_PWR_CLK_DISABLE();
    }
  }

#if defined(RCC_HSI48_SUPPORT)
  /*----------------------------- HSI48 Configuration --------------------------*/
  if(((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI48) == RCC_OSCILLATORTYPE_HSI48)
  {
      /* Check the HSI48 State */
      if(RCC_OscInitStruct->HSI48State != RCC_HSI48_OFF)
      {
        /* Enable the Internal High Speed oscillator (HSI48). */
        RCC_HSI48_ENABLE();

        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till HSI48 is ready */
        while(RCC_GET_FLAG(RCC_FLAG_HSI48RDY) == RESET)
        {
          if((GetTick() - tickstart) > HSI48_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }
      }
      else
      {
        /* Disable the Internal High Speed oscillator (HSI48). */
        RCC_HSI48_DISABLE();

        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till HSI48 is ready */
        while(RCC_GET_FLAG(RCC_FLAG_HSI48RDY) != RESET)
        {
          if((GetTick() - tickstart) > HSI48_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }
      }
  }
#endif /* RCC_HSI48_SUPPORT */

  /*-------------------------------- PLL Configuration -----------------------*/
  if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE)
  {
    /* Check if the PLL is used as system clock or not */
    if(RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK)
    {
      if((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON)
      {
        /* Disable the main PLL. */
        RCC_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till PLL is disabled */
        while(RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != RESET)
        {
          if((GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }

        /* Configure the main PLL clock source, multiplication and division factors. */
        RCC_PLL_CONFIG(RCC_OscInitStruct->PLL.PLLSource,
                             RCC_OscInitStruct->PLL.PLLMUL,
                             RCC_OscInitStruct->PLL.PLLDIV);
        /* Enable the main PLL. */
        RCC_PLL_ENABLE();

        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till PLL is ready */
        while(RCC_GET_FLAG(RCC_FLAG_PLLRDY)  == RESET)
        {
          if((GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }
      }
      else
      {
        /* Disable the main PLL. */
        RCC_PLL_DISABLE();

        /* Get Start Tick */
        tickstart = GetTick();

        /* Wait till PLL is disabled */
        while(RCC_GET_FLAG(RCC_FLAG_PLLRDY)  != RESET)
        {
          if((GetTick() - tickstart ) > PLL_TIMEOUT_VALUE)
          {
            return STATUS_TIMEOUT;
          }
        }
      }
    }
    else
    {
      return STATUS_ERROR;
    }
  }

  return STATUS_OK;
}

