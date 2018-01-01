/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
// Includes ------------------------------------------------------------------
#include <stm32l0xx.h>
#include "typedefs.h"
#include "main.h"
#include "macros.h"
#include "Configuration.h"
#include "SREGEmulation.h"
#include "rcc.h"
#include "serial.h"
#include "gpio.h"
#include "Conditionals.h"
#include "Planner.h"
#include <cstdlib>
#include "CommandParser.h"
#include "Stepper.h"
#include "Settings.h"
#include "Temperature.h"
#include <cstring>
#include "CommandProcessor.h"

// Private variables ---------------------------------------------------------

// USER CODE BEGIN PV
// Private variables ---------------------------------------------------------
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

			RCC_ClearFlag();

			Planner::init();

			CommandProcessor::init();

			// Load data from EEPROM if available (or use defaults)
			// This also updates variables in the planner, elsewhere
			Settings::load();

			thermalManager.init();    // Initialize temperature loop

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
			InitTick(TICK_INT_PRIORITY);

			// Init the low level hardware
			MspInit();
		}

		void MX_GPIO_Init(void)
		{
			// GPIO Ports Clock Enable
			//lint -save -e1924 -e9078 -e923 -e835 -e1960
			GPIO::RCC_GPIOA_CLK_ENABLE();
			GPIO::RCC_GPIOB_CLK_ENABLE();
			GPIO::RCC_GPIOC_CLK_ENABLE();
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
		SERIAL_ERROR_START();
		SERIAL_ERRORLNPGM(MSG_ERR_KILLED);

		Temperature::disable_all_heaters();
		disable_MOTOR();

		Timers::Delay(600U); // Wait a short time (allows messages to get out before shutting down.
		cli(); // Stop interrupts

		Timers::Delay(250U); //Wait to ensure all interrupts routines stopped
		Temperature::disable_all_heaters(); //turn off heaters again

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

namespace RuntimeSettings {
	// Initialized by settings.load()
	//lint -save -e1924
	bool axis_relative_modes = false;

	// Relative Mode.
	bool relative_mode = false;
	//lint -restore

	float32_t feedrate_mm_s = MMM_TO_MMS(1500.0F);

	int16_t feedrate_percentage = 100;
	int16_t saved_feedrate_percentage;
}

//lint -save -e9075 -e970
int main(void)
{
	Thermoprinter::BSP::Init();
	SystemClock_Config();
	Thermoprinter::BSP::MX_GPIO_Init();
	Thermoprinter::BSP::setup();
	//lint -save -e1924 -e716
	while (true) {
		CommandProcessor::process();

		#ifndef HOST_KEEPALIVE_FEATURE
			//lint -save -e522
		#endif
		Thermoprinter::idle();
		#ifndef HOST_KEEPALIVE_FEATURE
			//lint -restore
		#endif
	}
	//lint -restore
}
//lint -restore

extern "C" {
	void SysTick_Handler(void) {
	  IncTick();
	}
}

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
}

//*********************** (C) COPYRIGHT STMicroelectronics *****END OF FILE***
