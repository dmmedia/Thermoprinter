/*
 * Thermoprinter.cpp
 *
 *  Created on: 8. jaan 2018
 *      Author: Den
 */

#include <stdint.h>
#include <stm32l0xx.h>
#include "macros.h"
#include "Thermoprinter.h"
#include "Configuration.h"
#include "gpio.h"
#include "typedefs.h"
#include "Conditionals.h"
#include "serial.h"
#include "rcc.h"
#include "Planner.h"
#include "CommandProcessor.h"
#include "Settings.h"
#include "Temperature.h"
#include "Stepper.h"
#include "Timers.h"

namespace Thermoprinter {
	//lint -save -e1924
	bool Running = true;
	//lint -restore

	namespace BSP {
		void setup_powerhold() {
			if (hasPowerSwitch()) {
				//lint -save -e1924 -e835 -e9078 -e923
				GPIO::outWrite(PS_ON_PORT, PS_ON_PIN, PS_ON_AWAKE);
				//lint -restore
			}
		}

		//
		// Entry-point: Set up before the program loop
		//  - Set up the  power hold
		//  - Start the serial port
		//  - Print startup messages and diagnostics
		//  - Get EEPROM or default settings
		//  - Initialize managers for:
		//    • temperature
		//    • planner
		//    • watchdog
		//    • stepper
		//    • status LEDs
		//
		void setup() {
			setup_powerhold();

			MarlinSerial::begin(BAUDRATE);
			SERIAL_PROTOCOLLNPGM("start");

			SERIAL_ECHOPGM(MSG_THERMOPRINTER);
			SERIAL_CHAR(' ');
			SERIAL_ECHOLNPGM(SHORT_BUILD_VERSION);
			SERIAL_EOL();

			Rcc::RCC_ClearFlag();

			Planner::init();

			CommandProcessor::init();

			// Load data from EEPROM if available (or use defaults)
			// This also updates variables in the planner, elsewhere
			Settings::load();

			AdcManager::init();    // Initialize temperature loop

		  	#if defined(USE_WATCHDOG) && (USE_WATCHDOG > 0)
		    	watchdog_init();
		  	#endif

		    Stepper::init();    // Initialize stepper, this enables interrupts!

		  	#if defined(RGB_LED) || defined(RGBW_LED)
		    	SET_OUTPUT(RGB_LED_R_PIN);
		    	SET_OUTPUT(RGB_LED_G_PIN);
		    	SET_OUTPUT(RGB_LED_B_PIN);
		    	#if ENABLED(RGBW_LED)
		    		SET_OUTPUT(RGB_LED_W_PIN);
		    	#endif
		  	#endif
		}

		//
		// Initializes the Global MSP.
		///
		void MspInit(void)
		{
			//lint -save -e1924 -e9078 -e923 -e835 -e1960 -e9053
			RCC_SYSCFG_CLK_ENABLE();
			RCC_PWR_CLK_ENABLE();
			//lint -restore

			// System interrupt init
			// SVC_IRQn interrupt configuration
			NVIC_SetPriority(SVC_IRQn, 0U);
			// PendSV_IRQn interrupt configuration
			NVIC_SetPriority(PendSV_IRQn, 0U);
			// SysTick_IRQn interrupt configuration
			NVIC_SetPriority(SysTick_IRQn, 0U);
		}

		//
		// @brief This function configures the Flash prefetch, Flash preread and Buffer cache,
		//        Configures time base source, NVIC and Low level hardware
		// @note This function is called at the beginning of program after reset and before
		//       the clock configuration
		// @note The time base configuration is based on MSI clock when exiting from Reset.
		//       Once done, time base tick start incrementing.
		//        In the default implementation,Systick is used as source of time base.
		//        the tick variable is incremented each 1ms in its ISR.
		//
		void Init(void)
		{
			// Configure Buffer cache, Flash prefetch,  Flash preread
			//lint -save -e1924 -e9078 -e923
			SET_BIT((FLASH->ACR), FLASH_ACR_PRE_READ);
			//lint -restore

			// Use systick as time base source and configure 1ms tick (default clock after Reset is MSI)
			Timers::InitTick(TICK_INT_PRIORITY);

			// Init the low level hardware
			MspInit();
		}

		void MX_GPIO_Init(void)
		{
			// GPIO Ports Clock Enable
			//lint -save -e1924 -e9078 -e923 -e835 -e1960
			Rcc::RCC_GPIOA_CLK_ENABLE();
			Rcc::RCC_GPIOB_CLK_ENABLE();
			Rcc::RCC_GPIOC_CLK_ENABLE();
			//lint -restore
		}

	}

	//
	// Standard idle routine keeps the machine alive
	//
	void idle() {
		//lint -save -e950 -e1960
		host_keepalive();
		//lint -restore
	}

	//
	// Kill all activity and lock the machine.
	// After this the machine will need to be reset.
	//
	void kill() {
		Serial::SERIAL_ERROR_START();
		SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

		AdcManager::disable_all_heaters();
		Stepper::disable_MOTOR();

		Timers::Delay(600U); // Wait a short time (allows messages to get out before shutting down.
		__disable_irq(); // Stop interrupts

		Timers::Delay(250U); //Wait to ensure all interrupts routines stopped
		AdcManager::disable_all_heaters(); //turn off heaters again

		if (hasPowerSwitch()) {
			//lint -save -e1924 -e835 -e9078
			GPIO::setInput(PS_ON_PORT, PS_ON_PIN, GPIO::GPIO_MODE_INPUT);
			//lint -restore
		}

		//lint -save -e1924 -e716
		while (true) {
	    	#if defined(USE_WATCHDOG) && (USE_WATCHDOG > 0)
				watchdog_reset();
	    	#endif
		} // Wait for reset
		//lint -restore
	}

	//
	// @brief  This function is executed in case of error occurrence.
	// @param  None
	// @retval None
	//
	void Error_Handler(const char * const file, const int32_t line)
	{
		UNUSED(file);
		UNUSED(line);
		kill();
	}

	#if defined(HOST_KEEPALIVE_FEATURE) && (HOST_KEEPALIVE_FEATURE > 0)

	  	//
	  	// Output a "busy" message at regular intervals
	  	// while the machine is not accepting commands.
	  	//
	  	void host_keepalive() {
	  		const millis_t ms = millis();
	  		if (host_keepalive_interval && busy_state != NOT_BUSY) {
	  			if (PENDING(ms, next_busy_signal_ms)) return;
	  			switch (busy_state) {
	  				case IN_HANDLER:
	  				case IN_PROCESS:
	  					SERIAL_ECHO_START();
	  					SERIAL_ECHOLNPGM(MSG_BUSY_PROCESSING);
	  					break;
	  				case PAUSED_FOR_USER:
	  					SERIAL_ECHO_START();
	  					SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_USER);
	  					break;
	  				case PAUSED_FOR_INPUT:
	  					SERIAL_ECHO_START();
	  					SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_INPUT);
	  					break;
	  				default:
	  					break;
	  			}
	  		}
	  		next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
	  	}

	#endif // HOST_KEEPALIVE_FEATURE
}


