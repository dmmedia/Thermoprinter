#pragma once

#include <stm32l053xx.h>

namespace Thermoprinter {
	extern bool Running;
	inline bool IsRunning() { return  Running; }
	inline bool IsStopped() { return !Running; }

	void idle();

	namespace BSP {
		void MX_GPIO_Init(void);
		void setup_powerhold();
		void setup();
		void MspInit(void);
		void Init(void);
	}

	void kill();

	void Error_Handler(const char * const, const int32_t);

	#if defined(HOST_KEEPALIVE_FEATURE) && (HOST_KEEPALIVE_FEATURE > 0)
		MarlinBusyState busy_state = NOT_BUSY;
		static millis_t next_busy_signal_ms = 0;
		uint8_t host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
	#else
	  	#define host_keepalive() NOOP
	#endif

}

namespace RuntimeSettings {
	extern bool axis_relative_modes;
	extern bool relative_mode;

	extern float32_t feedrate_mm_s;

	extern int16_t feedrate_percentage;
	extern int16_t saved_feedrate_percentage;
}

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
