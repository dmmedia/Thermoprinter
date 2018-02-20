/*
 * Timers.cpp
 *
 *  Created on: 8. jaan 2018
 *      Author: Den
 */

#include <stm32l0xx.h>
#include "typedefs.h"
#include "Timers.h"
#include "Thermoprinter.h"
#include <stddef.h>

namespace Timers {
	//
	// Private constants and definitions
	//

	//
	// Variable initialization
	//

	//
	// @brief uwTick_variable uwTick variable
	//
	static __IO uint32_t uwTick;

	//
	// Private function protorypes
	//

	//
	// @brief This function is called to increment  a global variable "uwTick"
	//        used as application time base.
	// @note In the default implementation, this variable is incremented each 1ms
	//       in Systick ISR.
	// @retval None
	//
	static void IncTick(void);

	//
	// @brief  Time Base configuration
	// @param  TIMx : TIM peripheral
	// @param   Structure : TIM Base configuration structure
	// @retval None
	//
	static void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);

	//
	// Namespace body
	//

	//
	// @brief This function provides accurate delay (in ms) based on a variable incremented.
	// @note In the default implementation , SysTick timer is the source of time base.
	//       It is used to generate interrupts at regular time intervals where uwTick
	//       is incremented.
	// @param Delay: specifies the delay time length, in milliseconds.
	// @retval None
	//
	void Delay(uint32_t const delay_ms)
	{
		const uint32_t tickstart = GetTick();
		while((GetTick() - tickstart) < delay_ms)
		{
		}
	}

	//
	// @brief This function configures the source of the time base.
	//        The time source is configured  to have 1ms time base with a dedicated
	//        Tick interrupt priority.
	// @note This function is called  automatically at the beginning of program after
	//       reset by Init() or at any time when clock is reconfigured  by RCC_ClockConfig().
	// @note In the default implementation, SysTick timer is the source of time base.
	//       It is used to generate interrupts at regular time intervals.
	//       Care must be taken if HAL_Delay() is called from a peripheral ISR process,
	//       The the SysTick interrupt must have higher priority (numerically lower)
	//       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
	//       The function is declared as __Weak  to be overwritten  in case of other
	//       implementation  in user file.
	// @param TickPriority: Tick interrupt priority.
	//
	void InitTick(uint32_t const TickPriority) {
		// Configure the SysTick to have interrupt in 1ms time basis
		if (SysTick_Config(SystemCoreClock / 1000U) != 0U) {
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		// Configure the SysTick IRQ priority
		NVIC_SetPriority(SysTick_IRQn, TickPriority);
	}

	//
	// @brief Provides a tick value in millisecond.
	// @retval tick value
	//
	uint32_t GetTick(void) {
		return uwTick;
	}

	//
	// @brief This function is called to increment  a global variable "uwTick"
	//        used as application time base.
	// @note In the default implementation, this variable is incremented each 1ms
	//       in Systick ISR.
	// @retval None
	//
	static void IncTick(void) {
		uwTick++;
	}

	//
	// @brief  Time Base configuration
	// @param  TIMx : TIM peripheral
	// @param   Structure : TIM Base configuration structure
	// @retval None
	//
	static void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure)
	{
	  uint32_t tmpcr1 = 0U;
	  tmpcr1 = TIMx->CR1;

	  // Set TIM Time Base Unit parameters ---------------------------------------
	  if(IS_TIM_CC1_INSTANCE(TIMx) != RESET)
	  {
	    // Select the Counter Mode
	    tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
	    tmpcr1 |= Structure->CounterMode;
	  }

	  if(IS_TIM_CC1_INSTANCE(TIMx) != RESET)
	  {
	    // Set the clock division
	    tmpcr1 &= ~TIM_CR1_CKD;
	    tmpcr1 |= (uint32_t)Structure->ClockDivision;
	  }

	  TIMx->CR1 = tmpcr1;

	  // Set the Autoreload value
	  TIMx->ARR = (uint32_t)Structure->Period ;

	  // Set the Prescaler value
	  TIMx->PSC = (uint32_t)Structure->Prescaler;

	  // Generate an update event to reload the Prescaler value immediatly
	  TIMx->EGR = TIM_EGR_UG;
	}

	//
	// @brief  Initializes the TIM Time base Unit according to the specified
	//         parameters in the TIM_HandleTypeDef and create the associated handle.
	// @param  htim : TIM handle
	// @retval HAL status
	//
	StatusTypeDef TIM_Base_Init(TIM_HandleTypeDef *htim)
	{
	  // Check the TIM handle allocation
	  if(htim == NULL)
	  {
	    return STATUS_ERROR;
	  }

	  if(htim->State == TIM_STATE_RESET)
	  {
	    // Allocate lock resource and initialize it
	    htim->Lock = HANDLE_UNLOCKED;
	  }

	  // Set the TIM state
	  htim->State= TIM_STATE_BUSY;

	  // Set the Time Base configuration
	  TIM_Base_SetConfig(htim->Instance, &htim->Init);

	  // Initialize the TIM state
	  htim->State= TIM_STATE_READY;

	  return STATUS_OK;
	}

	// End

}

extern "C" {
	void SysTick_Handler(void) {
		Timers::IncTick();
	}
}
