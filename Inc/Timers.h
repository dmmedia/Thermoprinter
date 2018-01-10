#pragma once

/*
 * Timers.h
 *
 *  Created on: 8. jaan 2018
 *      Author: Den
 */

namespace Timers {
	//
	// @brief uwTick_variable uwTick variable
	//
	extern __IO uint32_t uwTick;

	//
	// @brief This function configures the source of the time base.
	//        The time source is configured  to have 1ms time base with a dedicated
	//        Tick interrupt priority.
	// @note This function is called  automatically at the beginning of program after
	//       reset by Init() or at any time when clock is reconfigured  by RCC_ClockConfig().
	// @note In the default implementation, SysTick timer is the source of time base.
	//       It is used to generate interrupts at regular time intervals.
	//       Care must be taken if Delay() is called from a peripheral ISR process,
	//       The the SysTick interrupt must have higher priority (numerically lower)
	//       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
	//       The function is declared as __Weak  to be overwritten  in case of other
	//       implementation  in user file.
	// @param TickPriority: Tick interrupt priority.
	//
	void InitTick(uint32_t TickPriority);

	void Delay(__IO uint32_t delay_ms);

	/**
	 * @brief Provides a tick value in millisecond.
	 * @retval tick value
	 */
	uint32_t GetTick(void);

	/**
	  * @brief This function is called to increment  a global variable "uwTick"
	  *        used as application time base.
	  * @note In the default implementation, this variable is incremented each 1ms
	  *       in Systick ISR.
	  * @retval None
	  */
	void IncTick(void);

}

extern "C" {
	void SysTick_Handler(void);
}
