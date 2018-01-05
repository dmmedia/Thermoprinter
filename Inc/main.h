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
	void Delay(__IO uint32_t delay_ms);
	uint32_t GetTick(void);
}

extern "C" {
	void SysTick_Handler(void);
}
