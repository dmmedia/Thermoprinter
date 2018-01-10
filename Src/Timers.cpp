/*
 * Timers.cpp
 *
 *  Created on: 8. jaan 2018
 *      Author: Den
 */

#include "Timers.h"

namespace Timers {
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
	// @brief uwTick_variable uwTick variable
	//
	volatile uint32_t uwTick;

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
	void IncTick(void) {
		uwTick++;
	}

}

extern "C" {
	void SysTick_Handler(void) {
		Timers::IncTick();
	}
}
