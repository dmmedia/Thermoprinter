#pragma once

/*
 * Thermoprinter.h
 *
 *  Created on: 8. jaan 2018
 *      Author: Den
 */

namespace Thermoprinter {
	extern bool Running;
	bool IsRunning();
	bool IsStopped();

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

