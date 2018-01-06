/*
 * rcc.cpp
 *
 *  Created on: 17. dets 2017
 *      Author: Den
 */

#include <stm32l0xx.h>
#include "typedefs.h"
#include "macros.h"
#include "main.h"
#include "rcc.h"

namespace Rcc {
	//
	// @brief uwTick_variable uwTick variable
	//
	__IO uint32_t uwTick;

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

	//
	// @brief  Returns the SYSCLK frequency
	// @note   The system frequency computed by this function is not the real
	//         frequency in the chip. It is calculated based on the predefined
	//         constant and the selected clock source:
	// @note     If SYSCLK source is MSI, function returns a value based on MSI
	//             Value as defined by the MSI range.
	// @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
	// @note     If SYSCLK source is HSE, function returns a value based on HSE_VALUE(**)
	// @note     If SYSCLK source is PLL, function returns a value based on HSE_VALUE(**)
	//           or HSI_VALUE(*) multiplied/divided by the PLL factors.
	// @note     (*) HSI_VALUE is a constant defined in stm32l0xx_hal_conf.h file (default value
	//               16 MHz) but the real value may vary depending on the variations
	//               in voltage and temperature.
	// @note     (**) HSE_VALUE is a constant defined in stm32l0xx_hal_conf.h file (default value
	//                8 MHz), user has to ensure that HSE_VALUE is same as the real
	//                frequency of the crystal used. Otherwise, this function may
	//                have wrong result.
	//
	// @note   The result of this function could be not correct when using fractional
	//         value for HSE crystal.
	//
	// @note   This function can be used by the user application to compute the
	//         baud-rate for the communication peripherals or configure other parameters.
	//
	// @note   Each time SYSCLK changes, this function must be called to update the
	//         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
	//
	// @retval SYSCLK frequency
	//
	uint32_t RCC_GetSysClockFreq(void) {
		uint32_t pllm = 0U;
		uint32_t plld = 0U;
		uint32_t pllvco = 0U;
		uint32_t msiclkrange = 0U;
		uint32_t sysclockfreq = 0U;

		//lint -save -e1924 -e9078 -e923
		uint32_t tmpreg = RCC->CFGR;
		//lint -restore

		// Get SYSCLK source -------------------------------------------------------
		switch (tmpreg & RCC_CFGR_SWS) {
			case RCC_SYSCLKSOURCE_STATUS_HSI:  // HSI used as system clock source
			{
				//lint -save -e1924 -e9078 -e923
				if ((RCC->CR & RCC_CR_HSIDIVF) != 0U) {
					sysclockfreq = (HSI_VALUE >> 2);
				} else {
					sysclockfreq = HSI_VALUE;
				}
				//lint -restore
				break;
			}
			case RCC_SYSCLKSOURCE_STATUS_HSE:  // HSE used as system clock
			{
				sysclockfreq = HSE_VALUE;
				break;
			}
			case RCC_SYSCLKSOURCE_STATUS_PLLCLK:  // PLL used as system clock
			{
				pllm = PLLMulTable[(tmpreg & RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_BITNUMBER];
				plld = ((tmpreg & RCC_CFGR_PLLDIV) >> RCC_CFGR_PLLDIV_BITNUMBER) + 1;
				if (RCC_GET_PLL_OSCSOURCE() != RCC_PLLSOURCE_HSI) {
					// HSE used as PLL clock source
					pllvco = (HSE_VALUE * pllm) / plld;
				} else {
					if ((RCC->CR & RCC_CR_HSIDIVF) != 0) {
						pllvco = ((HSI_VALUE >> 2) * pllm) / plld;
					} else {
						pllvco = (HSI_VALUE * pllm) / plld;
					}
				}
				sysclockfreq = pllvco;
				break;
			}
			case RCC_SYSCLKSOURCE_STATUS_MSI:  // MSI used as system clock source
			default: // MSI used as system clock
			{
				msiclkrange = (RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> RCC_ICSCR_MSIRANGE_BITNUMBER;
				sysclockfreq = (32768 * (1 << (msiclkrange + 1)));
				break;
			}
		}
		return sysclockfreq;
	}

	//
	// @brief  Initializes the RCC Oscillators according to the specified parameters in the
	//         RCC_OscInitTypeDef.
	// @param  RCC_OscInitStruct pointer to an RCC_OscInitTypeDef structure that
	//         contains the configuration information for the RCC Oscillators.
	// @note   The PLL is not disabled when used as system clock.
	// @note   Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not
	//         supported by this macro. User should request a transition to LSE Off
	//         first and then LSE On or LSE Bypass.
	// @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
	//         supported by this macro. User should request a transition to HSE Off
	//         first and then HSE On or HSE Bypass.
	// @retval status
	//
	StatusTypeDef RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct) {
		uint32_t tickstart = 0U;

		//------------------------------- HSE Configuration ------------------------
		if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSE) ==
		RCC_OSCILLATORTYPE_HSE) {
			// When the HSE is used as system clock or clock source for PLL in these cases it is not allowed to be disabled
			if ((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSE)
				|| ((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK)
					&& (RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSE))) {
				if ((RCC_GetFlag(RCC_FLAG_HSERDY) != RESET)
					&& (RCC_OscInitStruct->HSEState == RCC_HSE_OFF)) {
					return STATUS_ERROR;
				}
			} else {
				// Set the new HSE configuration ---------------------------------------
				RCC_HSE_CONFIG(RCC_OscInitStruct->HSEState);

				// Check the HSE State
				if (RCC_OscInitStruct->HSEState != RCC_HSE_OFF) {
					// Get Start Tick
					tickstart = GetTick();

					// Wait till HSE is ready
					while (RCC_GetFlag(RCC_FLAG_HSERDY) == RESET) {
						if ((GetTick() - tickstart) > HSE_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}
				} else {
					// Get Start Tick
					tickstart = GetTick();

					// Wait till HSE is disabled
					while (RCC_GetFlag(RCC_FLAG_HSERDY) != RESET) {
						if ((GetTick() - tickstart) > HSE_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}
				}
			}
		}
		//----------------------------- HSI Configuration --------------------------
		if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI) == RCC_OSCILLATORTYPE_HSI) {
			// Check if HSI is used as system clock or as PLL source when PLL is selected as system clock
			if ((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_HSI)
				|| ((RCC_GET_SYSCLK_SOURCE() == RCC_SYSCLKSOURCE_STATUS_PLLCLK)
					&& (RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSI))) {
				// When HSI is used as system clock it will not disabled
				if ((RCC_GetFlag(RCC_FLAG_HSIRDY) != RESET)
					&& (RCC_OscInitStruct->HSIState != RCC_HSI_ON)) {
					return STATUS_ERROR;
				}
				// Otherwise, just the calibration is allowed
				else {
					// Adjusts the Internal High Speed oscillator (HSI) calibration value.
					RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
				}
			} else {
				// Check the HSI State
				if (RCC_OscInitStruct->HSIState != RCC_HSI_OFF) {
					// Enable the Internal High Speed oscillator (HSI or HSIdiv4)
					RCC_HSI_CONFIG(RCC_OscInitStruct->HSIState);

					// Get Start Tick
					tickstart = GetTick();

					// Wait till HSI is ready
					while (RCC_GetFlag(RCC_FLAG_HSIRDY) == RESET) {
						if ((GetTick() - tickstart) > HSI_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}

					// Adjusts the Internal High Speed oscillator (HSI) calibration value.
					RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
				} else {
					// Disable the Internal High Speed oscillator (HSI).
					RCC_HSI_DISABLE();

					// Get Start Tick
					tickstart = GetTick();

					// Wait till HSI is disabled
					while (RCC_GetFlag(RCC_FLAG_HSIRDY) != RESET) {
						if ((GetTick() - tickstart) > HSI_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}
				}
			}
		}
		//----------------------------- MSI Configuration --------------------------
		if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_MSI) == RCC_OSCILLATORTYPE_MSI) {
			// When the MSI is used as system clock it will not be disabled
			if ((RCC_GET_SYSCLK_SOURCE() == RCC_CFGR_SWS_MSI)) {
				if ((RCC_GetFlag(RCC_FLAG_MSIRDY) != RESET)
					&& (RCC_OscInitStruct->MSIState == RCC_MSI_OFF)) {
					return STATUS_ERROR;
				}
				// Otherwise, just the calibration and MSI range change are allowed
				else {
					// To correctly read data from FLASH memory, the number of wait states (LATENCY)
					// must be correctly programmed according to the frequency of the CPU clock
					// (HCLK) and the supply voltage of the device.
					if (RCC_OscInitStruct->MSIClockRange > RCC_GET_MSI_RANGE()) {
						// First increase number of wait states update if necessary
						if (RCC_SetFlashLatencyFromMSIRange(RCC_OscInitStruct->MSIClockRange)
							!= STATUS_OK) {
							return STATUS_ERROR;
						}

						// Selects the Multiple Speed oscillator (MSI) clock range .
						RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);
						// Adjusts the Multiple Speed oscillator (MSI) calibration value.
						RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);
					} else {
						// Else, keep current flash latency while decreasing applies
						// Selects the Multiple Speed oscillator (MSI) clock range.
						RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);
						// Adjusts the Multiple Speed oscillator (MSI) calibration value.
						RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);

						// Decrease number of wait states update if necessary
						if (RCC_SetFlashLatencyFromMSIRange(RCC_OscInitStruct->MSIClockRange)
							!= STATUS_OK) {
							return STATUS_ERROR;
						}
					}

					// Update the SystemCoreClock global variable
					SystemCoreClock = (32768U
						* (1U
							<< ((RCC_OscInitStruct->MSIClockRange >> RCC_ICSCR_MSIRANGE_BITNUMBER)
								+ 1U)))
						>> AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_BITNUMBER)];

					// Configure the source of time base considering new system clocks settings
					InitTick(TICK_INT_PRIORITY);
				}
			} else {
				// Check the MSI State
				if (RCC_OscInitStruct->MSIState != RCC_MSI_OFF) {
					// Enable the Multi Speed oscillator (MSI).
					RCC_MSI_ENABLE();

					// Get Start Tick
					tickstart = GetTick();

					// Wait till MSI is ready
					while (RCC_GetFlag(RCC_FLAG_MSIRDY) == RESET) {
						if ((GetTick() - tickstart) > MSI_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}
					// Selects the Multiple Speed oscillator (MSI) clock range.
					RCC_MSI_RANGE_CONFIG(RCC_OscInitStruct->MSIClockRange);
					// Adjusts the Multiple Speed oscillator (MSI) calibration value.
					RCC_MSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->MSICalibrationValue);

				} else {
					// Disable the Multi Speed oscillator (MSI).
					RCC_MSI_DISABLE();

					// Get Start Tick
					tickstart = GetTick();

					// Wait till MSI is ready
					while (RCC_GetFlag(RCC_FLAG_MSIRDY) != RESET) {
						if ((GetTick() - tickstart) > MSI_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}
				}
			}
		}
		//------------------------------ LSI Configuration -------------------------
		if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSI) == RCC_OSCILLATORTYPE_LSI) {
			// Check the LSI State
			if (RCC_OscInitStruct->LSIState != RCC_LSI_OFF) {
				// Enable the Internal Low Speed oscillator (LSI).
				RCC_LSI_ENABLE();

				// Get Start Tick
				tickstart = GetTick();

				// Wait till LSI is ready
				while (RCC_GetFlag(RCC_FLAG_LSIRDY) == RESET) {
					if ((GetTick() - tickstart) > LSI_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else {
				// Disable the Internal Low Speed oscillator (LSI).
				RCC_LSI_DISABLE();

				// Get Start Tick
				tickstart = GetTick();

				// Wait till LSI is disabled
				while (RCC_GetFlag(RCC_FLAG_LSIRDY) != RESET) {
					if ((GetTick() - tickstart) > LSI_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			}
		}
		//------------------------------ LSE Configuration -------------------------
		if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_LSE) == RCC_OSCILLATORTYPE_LSE) {
			FlagStatus pwrclkchanged = RESET;

			// Update LSE configuration in Backup Domain control register
			// Requires to enable write access to Backup Domain of necessary
			if (RCC_PWR_IS_CLK_DISABLED()) {
				RCC_PWR_CLK_ENABLE();
				pwrclkchanged = SET;
			}

			if (IS_BIT_CLR(PWR->CR, PWR_CR_DBP)) {
				// Enable write access to Backup domain
				SET_BIT(PWR->CR, PWR_CR_DBP);

				// Wait for Backup domain Write protection disable
				tickstart = GetTick();

				while (IS_BIT_CLR(PWR->CR, PWR_CR_DBP)) {
					if ((GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			}

			// Set the new LSE configuration -----------------------------------------
			RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);
			// Check the LSE State
			if (RCC_OscInitStruct->LSEState != RCC_LSE_OFF) {
				// Get Start Tick
				tickstart = GetTick();

				// Wait till LSE is ready
				while (RCC_GetFlag(RCC_FLAG_LSERDY) == RESET) {
					if ((GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else {
				// Get Start Tick
				tickstart = GetTick();

				// Wait till LSE is disabled
				while (RCC_GetFlag(RCC_FLAG_LSERDY) != RESET) {
					if ((GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			}

			// Require to disable power clock if necessary
			if (pwrclkchanged == SET) {
				RCC_PWR_CLK_DISABLE();
			}
		}

#if defined(RCC_HSI48_SUPPORT)
		//----------------------------- HSI48 Configuration --------------------------
		if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI48)
			== RCC_OSCILLATORTYPE_HSI48) {
			// Check the HSI48 State
			if (RCC_OscInitStruct->HSI48State != RCC_HSI48_OFF) {
				// Enable the Internal High Speed oscillator (HSI48).
				RCC_HSI48_ENABLE()
				;

				// Get Start Tick
				tickstart = GetTick();

				// Wait till HSI48 is ready
				while (RCC_GetFlag(RCC_FLAG_HSI48RDY) == RESET) {
					if ((GetTick() - tickstart) > HSI48_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else {
				// Disable the Internal High Speed oscillator (HSI48).
				RCC_HSI48_DISABLE()
				;

				// Get Start Tick
				tickstart = GetTick();

				// Wait till HSI48 is ready
				while (RCC_GetFlag(RCC_FLAG_HSI48RDY) != RESET) {
					if ((GetTick() - tickstart) > HSI48_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			}
		}
#endif // RCC_HSI48_SUPPORT

		//-------------------------------- PLL Configuration -----------------------
		if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE) {
			// Check if the PLL is used as system clock or not
			if (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK) {
				if ((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON) {
					// Disable the main PLL.
					RCC_PLL_DISABLE();

					// Get Start Tick
					tickstart = GetTick();

					// Wait till PLL is disabled
					while (RCC_GetFlag(RCC_FLAG_PLLRDY) != RESET) {
						if ((GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}

					// Configure the main PLL clock source, multiplication and division factors.
					RCC_PLL_CONFIG(RCC_OscInitStruct->PLL.PLLSource, RCC_OscInitStruct->PLL.PLLMUL,
						RCC_OscInitStruct->PLL.PLLDIV);
					// Enable the main PLL.
					RCC_PLL_ENABLE();

					// Get Start Tick
					tickstart = GetTick();

					// Wait till PLL is ready
					while (RCC_GetFlag(RCC_FLAG_PLLRDY) == RESET) {
						if ((GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}
				} else {
					// Disable the main PLL.
					RCC_PLL_DISABLE();

					// Get Start Tick
					tickstart = GetTick();

					// Wait till PLL is disabled
					while (RCC_GetFlag(RCC_FLAG_PLLRDY) != RESET) {
						if ((GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}
				}
			} else {
				return STATUS_ERROR;
			}
		}

		return STATUS_OK;
	}

	void RCC_ClearFlag(void) {
		// Set RMVF bit to clear the reset flags
		RCC->CSR |= RCC_CSR_RMVF;
	}

	//
	// @brief  Initializes the CPU, AHB and APB buses clocks according to the specified
	//         parameters in the RCC_ClkInitStruct.
	// @param  RCC_ClkInitStruct pointer to an RCC_OscInitTypeDef structure that
	//         contains the configuration information for the RCC peripheral.
	// @param  FLatency FLASH Latency
	//          The value of this parameter depend on device used within the same series
	// @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
	//         and updated by @ref HAL_RCC_GetHCLKFreq() function called within this function
	//
	// @note   The MSI is used (enabled by hardware) as system clock source after
	//         start-up from Reset, wake-up from STOP and STANDBY mode, or in case
	//         of failure of the HSE used directly or indirectly as system clock
	//         (if the Clock Security System CSS is enabled).
	//
	// @note   A switch from one clock source to another occurs only if the target
	//         clock source is ready (clock stable after start-up delay or PLL locked).
	//         If a clock source which is not yet ready is selected, the switch will
	//         occur when the clock source will be ready.
	//         You can use @ref HAL_RCC_GetClockConfig() function to know which clock is
	//         currently used as system clock source.
	// @note   Depending on the device voltage range, the software has to set correctly
	//         HPRE[3:0] bits to ensure that HCLK not exceed the maximum allowed frequency
	//         (for more details refer to section above "Initialization/de-initialization functions")
	// @retval HAL status
	//
	StatusTypeDef RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency) {
		uint32_t tickstart = 0U;

		// To correctly read data from FLASH memory, the number of wait states (LATENCY)
		// must be correctly programmed according to the frequency of the CPU clock
		// (HCLK) and the supply voltage of the device.

		// Increasing the number of wait states because of higher CPU frequency
		if (FLatency > (FLASH->ACR & FLASH_ACR_LATENCY)) {
			// Program the new number of wait states to the LATENCY bits in the FLASH_ACR register
			FLASH_SET_LATENCY(FLatency);

			// Check that the new number of wait states is taken into account to access the Flash
			// memory by reading the FLASH_ACR register
			if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLatency) {
				return STATUS_ERROR;
			}
		}

		//-------------------------- HCLK Configuration --------------------------
		if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_HCLK) == RCC_CLOCKTYPE_HCLK) {
			MODIFY_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);
		}

		//------------------------- SYSCLK Configuration ---------------------------
		if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_SYSCLK) == RCC_CLOCKTYPE_SYSCLK) {
			// HSE is selected as System Clock Source
			if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE) {
				// Check the HSE ready flag
				if (RCC_GetFlag(RCC_FLAG_HSERDY) == RESET) {
					return STATUS_ERROR;
				}
			}
			// PLL is selected as System Clock Source
			else if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK) {
				// Check the PLL ready flag
				if (RCC_GetFlag(RCC_FLAG_PLLRDY) == RESET) {
					return STATUS_ERROR;
				}
			}
			// HSI is selected as System Clock Source
			else if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSI) {
				// Check the HSI ready flag
				if (RCC_GetFlag(RCC_FLAG_HSIRDY) == RESET) {
					return STATUS_ERROR;
				}
			}
			// MSI is selected as System Clock Source
			else {
				// Check the MSI ready flag
				if (RCC_GetFlag(RCC_FLAG_MSIRDY) == RESET) {
					return STATUS_ERROR;
				}
			}
			RCC_SYSCLK_CONFIG(RCC_ClkInitStruct->SYSCLKSource);

			// Get Start Tick
			tickstart = GetTick();

			if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE) {
				while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSE) {
					if ((GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK) {
				while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK) {
					if ((GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSI) {
				while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSI) {
					if ((GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else {
				while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_MSI) {
					if ((GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			}
		}
		// Decreasing the number of wait states because of lower CPU frequency
		if (FLatency < (FLASH->ACR & FLASH_ACR_LATENCY)) {
			// Program the new number of wait states to the LATENCY bits in the FLASH_ACR register
			FLASH_SET_LATENCY(FLatency);

			// Check that the new number of wait states is taken into account to access the Flash
			// memory by reading the FLASH_ACR register
			if ((FLASH->ACR & FLASH_ACR_LATENCY) != FLatency) {
				return STATUS_ERROR;
			}
		}

		//-------------------------- PCLK1 Configuration ---------------------------
		if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK1) == RCC_CLOCKTYPE_PCLK1) {
			MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);
		}

		//-------------------------- PCLK2 Configuration ---------------------------
		if (((RCC_ClkInitStruct->ClockType) & RCC_CLOCKTYPE_PCLK2) == RCC_CLOCKTYPE_PCLK2) {
			MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE2, ((RCC_ClkInitStruct->APB2CLKDivider) << 3));
		}

		// Update the SystemCoreClock global variable
		SystemCoreClock = RCC_GetSysClockFreq()
			>> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_BITNUMBER];

		// Configure the source of time base considering new system clocks settings
		InitTick(TICK_INT_PRIORITY);

		return STATUS_OK;
	}

	//
	// @brief  Initializes the RCC extended peripherals clocks according to the specified
	//         parameters in the RCC_PeriphCLKInitTypeDef.
	// @param  PeriphClkInit pointer to an RCC_PeriphCLKInitTypeDef structure that
	//         contains the configuration information for the Extended Peripherals clocks(USART1,USART2, LPUART1,
	//         I2C1, I2C3, RTC, USB/RNG  and LPTIM1 clocks).
	// @retval status
	// @note   If STATUS_ERROR returned, first switch-OFF HSE clock oscillator with @ref RCC_OscConfig()
	//         to possibly update HSE divider.
	//
	StatusTypeDef RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef *PeriphClkInit) {
		uint32_t tickstart = 0U;
		uint32_t temp_reg = 0U;

		//------------------------------- RTC/LCD Configuration ------------------------
		if ((((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC) == RCC_PERIPHCLK_RTC)
#if defined(LCD)
			|| (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LCD) == RCC_PERIPHCLK_LCD)
#endif // LCD
			) {
			FlagStatus pwrclkchanged = RESET;

			// As soon as function is called to change RTC clock source, activation of the
			// power domain is done.
			// Requires to enable write access to Backup Domain of necessary
			if (RCC_PWR_IS_CLK_DISABLED()) {
				RCC_PWR_CLK_ENABLE();
				pwrclkchanged = SET;
			}

			if (IS_BIT_CLR(PWR->CR, PWR_CR_DBP)) {
				// Enable write access to Backup domain
				SET_BIT(PWR->CR, PWR_CR_DBP);

				// Wait for Backup domain Write protection disable
				tickstart = GetTick();

				while (IS_BIT_CLR(PWR->CR, PWR_CR_DBP)) {
					if ((GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			}

			// Check if user wants to change HSE RTC prescaler whereas HSE is enabled
			temp_reg = (RCC->CR & RCC_CR_RTCPRE);
			if ((temp_reg != (PeriphClkInit->RTCClockSelection & RCC_CR_RTCPRE))
#if defined (LCD)
				|| (temp_reg != (PeriphClkInit->LCDClockSelection & RCC_CR_RTCPRE))
#endif // LCD
				) { // Check HSE State
				if (((PeriphClkInit->RTCClockSelection & RCC_CSR_RTCSEL) == RCC_CSR_RTCSEL_HSE)
					&& isBitSet(RCC->CR, RCC_CR_HSERDY)) {
					// To update HSE divider, first switch-OFF HSE clock oscillator
					return STATUS_ERROR;
				}
			}

			// Reset the Backup domain only if the RTC Clock source selection is modified from reset value
			temp_reg = (RCC->CSR & RCC_CSR_RTCSEL);

			if ((temp_reg != 0x00000000U)
				&& (((temp_reg != (PeriphClkInit->RTCClockSelection & RCC_CSR_RTCSEL))
					&& (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_RTC)
						== RCC_PERIPHCLK_RTC))
#if defined(LCD)
					|| ((temp_reg != (PeriphClkInit->LCDClockSelection & RCC_CSR_RTCSEL))
						&& (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LCD)
							== RCC_PERIPHCLK_LCD))
#endif // LCD
				)) {
				// Store the content of CSR register before the reset of Backup Domain
				temp_reg = (RCC->CSR & ~(RCC_CSR_RTCSEL));

				// RTC Clock selection can be changed only if the Backup Domain is reset
				RCC_BACKUPRESET_FORCE();
				RCC_BACKUPRESET_RELEASE();

				// Restore the Content of CSR register
				RCC->CSR = temp_reg;

				// Wait for LSERDY if LSE was enabled
				if (isBitSet(temp_reg, RCC_CSR_LSEON)) {
					// Get Start Tick
					tickstart = GetTick();

					// Wait till LSE is ready
					while (RCC_GetFlag(RCC_FLAG_LSERDY) == RESET) {
						if ((GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}
				}
			}
			RCC_RTC_CONFIG(PeriphClkInit->RTCClockSelection);

			// Require to disable power clock if necessary
			if (pwrclkchanged == SET) {
				RCC_PWR_CLK_DISABLE();
			}
		}

#if defined (RCC_CCIPR_USART1SEL)
		//------------------------------- USART1 Configuration ------------------------
		if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART1) == RCC_PERIPHCLK_USART1) {
			// Configure the USART1 clock source
			RCC_USART1_CONFIG(PeriphClkInit->Usart1ClockSelection);
		}
#endif // RCC_CCIPR_USART1SEL

		//----------------------------- USART2 Configuration --------------------------
		if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART2) == RCC_PERIPHCLK_USART2) {
			// Configure the USART2 clock source
			RCC_USART2_CONFIG(PeriphClkInit->Usart2ClockSelection);
		}

		//------------------------------ LPUART1 Configuration ------------------------
		if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPUART1) == RCC_PERIPHCLK_LPUART1) {
			// Configure the LPUAR1 clock source
			RCC_LPUART1_CONFIG(PeriphClkInit->Lpuart1ClockSelection);
		}

		//------------------------------ I2C1 Configuration ------------------------
		if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_I2C1) == RCC_PERIPHCLK_I2C1) {
			// Configure the I2C1 clock source
			RCC_I2C1_CONFIG(PeriphClkInit->I2c1ClockSelection);
		}

#if defined(USB)
		//---------------------------- USB and RNG configuration --------------------
		if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USB) == (RCC_PERIPHCLK_USB)) {
			RCC_USB_CONFIG(PeriphClkInit->UsbClockSelection);
		}
#endif // USB

		//---------------------------- LPTIM1 configuration ------------------------
		if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LPTIM1)
			== (RCC_PERIPHCLK_LPTIM1)) {
			RCC_LPTIM1_CONFIG(PeriphClkInit->LptimClockSelection);
		}

		return STATUS_OK;
	}

	//
	// @brief  Configures the SysTick clock source.
	// @param  CLKSource: specifies the SysTick clock source.
	//          This parameter can be one of the following values:
	//             @arg SYSTICK_CLKSOURCE_HCLK_DIV8: AHB clock divided by 8 selected as SysTick clock source.
	//             @arg SYSTICK_CLKSOURCE_HCLK: AHB clock selected as SysTick clock source.
	// @retval None
	//
	void SYSTICK_CLKSourceConfig(uint32_t CLKSource) {
		if (CLKSource == SYSTICK_CLKSOURCE_HCLK) {
			SysTick->CTRL |= SYSTICK_CLKSOURCE_HCLK;
		} else {
			SysTick->CTRL &= ~SYSTICK_CLKSOURCE_HCLK;
		}
	}

	// System Clock Configuration
	//
	void SystemClock_Config(void) {

		RCC_OscInitTypeDef RCC_OscInitStruct;
		RCC_ClkInitTypeDef RCC_ClkInitStruct;
		RCC_PeriphCLKInitTypeDef PeriphClkInit;

		//Configure the main internal regulator output voltage
		//
		(MODIFY_REG(PWR->CR, PWR_CR_VOS, (PWR_REGULATOR_VOLTAGE_SCALE1)));

		//Initializes the CPU, AHB and APB busses clocks
		//
		RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		RCC_OscInitStruct.HSIState = RCC_HSI_ON;
		RCC_OscInitStruct.HSICalibrationValue = 16;
		RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
		if (RCC_OscConfig(&RCC_OscInitStruct) != STATUS_OK) {
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		//Initializes the CPU, AHB and APB busses clocks
		//
		RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
		RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
		RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
		RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

		if (RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != STATUS_OK) {
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_LPUART1;
		PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
		PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
		if (RCCEx_PeriphCLKConfig(&PeriphClkInit) != STATUS_OK) {
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		//Configure the Systick
		//
		SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	}

	//
	// @brief  Returns the PCLK1 frequency
	// @note   Each time PCLK1 changes, this function must be called to update the
	//         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
	// @retval PCLK1 frequency
	//
	uint32_t RCC_GetPCLK1Freq(void) {
		// Get HCLK source and Compute PCLK1 frequency ---------------------------
		return (SystemCoreClock
			>> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_BITNUMBER]);
	}

	//
	// @brief  Returns the PCLK2 frequency
	// @note   Each time PCLK2 changes, this function must be called to update the
	//         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
	// @retval PCLK2 frequency
	//
	uint32_t RCC_GetPCLK2Freq(void) {
		// Get HCLK source and Compute PCLK2 frequency ---------------------------
		return (SystemCoreClock
			>> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_BITNUMBER]);
	}

	//
	// @brief  Return the peripheral clock frequency
	// @note   Return 0 if peripheral clock is unknown
	// @param  PeriphClk Peripheral clock identifier
	//         This parameter can be one of the following values:
	//            @arg @ref RCC_PERIPHCLK_RTC      RTC peripheral clock
	//            @arg @ref RCC_PERIPHCLK_LCD      LCD peripheral clock (*)
	//            @arg @ref RCC_PERIPHCLK_USB      USB or RNG peripheral clock (*)
	//            @arg @ref RCC_PERIPHCLK_USART1   USART1 peripheral clock (*)
	//            @arg @ref RCC_PERIPHCLK_USART2   USART2 peripheral clock
	//            @arg @ref RCC_PERIPHCLK_LPUART1  LPUART1 peripheral clock
	//            @arg @ref RCC_PERIPHCLK_I2C1     I2C1 peripheral clock
	//            @arg @ref RCC_PERIPHCLK_I2C2     I2C2 peripheral clock (*)
	//            @arg @ref RCC_PERIPHCLK_I2C3     I2C3 peripheral clock (*)
	// @note   (*) means that this peripheral is not present on all the devices
	// @retval Frequency in Hz (0: means that no available frequency for the peripheral)
	//
	uint32_t RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk) {
		uint32_t frequency = 0U;
		uint32_t srcclk = 0U;
#if defined(USB)
		uint32_t pllmul = 0U, plldiv = 0U, pllvco = 0U;
#endif // USB

		switch (PeriphClk) {
			case RCC_PERIPHCLK_RTC:
#if defined(USB)
			case RCC_PERIPHCLK_USB: {
				// Get the current USB source
				srcclk = RCC_GET_USB_SOURCE();

				if ((srcclk == RCC_USBCLKSOURCE_PLL) && (isBitSet(RCC->CR, RCC_CR_PLLRDY))) {
					// Get PLL clock source and multiplication factor ----------------------
					pllmul = RCC->CFGR & RCC_CFGR_PLLMUL;
					plldiv = RCC->CFGR & RCC_CFGR_PLLDIV;
					pllmul = PLLMulTable[(pllmul >> RCC_CFGR_PLLMUL_Pos)];
					plldiv = (plldiv >> RCC_CFGR_PLLDIV_Pos) + 1U;

					// Compute PLL clock input
					if (RCC_GET_PLL_OSCSOURCE() == RCC_PLLSOURCE_HSI) {
						if (READ_BIT(RCC->CR, RCC_CR_HSIDIVF) != 0U) {
							pllvco = (HSI_VALUE >> 2U);
						} else {
							pllvco = HSI_VALUE;
						}
					} else // HSE source
					{
						pllvco = HSE_VALUE;
					}
					// pllvco * pllmul / plldiv
					pllvco = (pllvco * pllmul);
					frequency = (pllvco / plldiv);

				} else if ((srcclk == RCC_USBCLKSOURCE_HSI48)
					&& (isBitSet(RCC->CRRCR, RCC_CRRCR_HSI48RDY))) {
					frequency = HSI48_VALUE;
				} else // RCC_USBCLKSOURCE_NONE
				{
					frequency = 0U;
				}
				break;
			}
#endif // USB
#if defined(RCC_CCIPR_USART1SEL)
			case RCC_PERIPHCLK_USART1: {
				// Get the current USART1 source
				srcclk = RCC_GET_USART1_SOURCE();

				// Check if USART1 clock selection is PCLK2
				if (srcclk == RCC_USART1CLKSOURCE_PCLK2) {
					frequency = RCC_GetPCLK2Freq();
				}
				// Check if HSI is ready and if USART1 clock selection is HSI
				else if ((srcclk == RCC_USART1CLKSOURCE_HSI)
					&& (isBitSet(RCC->CR, RCC_CR_HSIRDY))) {
					frequency = HSI_VALUE;
				}
				// Check if USART1 clock selection is SYSCLK
				else if (srcclk == RCC_USART1CLKSOURCE_SYSCLK) {
					frequency = RCC_GetSysClockFreq();
				}
				// Check if LSE is ready  and if USART1 clock selection is LSE
				else if ((srcclk == RCC_USART1CLKSOURCE_LSE)
					&& (isBitSet(RCC->CSR, RCC_CSR_LSERDY))) {
					frequency = LSE_VALUE;
				}
				// Clock not enabled for USART1
				else {
					frequency = 0U;
				}
				break;
			}
#endif // RCC_CCIPR_USART1SEL
			case RCC_PERIPHCLK_USART2: {
				// Get the current USART2 source
				srcclk = RCC_GET_USART2_SOURCE();

				// Check if USART2 clock selection is PCLK1
				if (srcclk == RCC_USART2CLKSOURCE_PCLK1) {
					frequency = RCC_GetPCLK1Freq();
				}
				// Check if HSI is ready and if USART2 clock selection is HSI
				else if ((srcclk == RCC_USART2CLKSOURCE_HSI)
					&& (isBitSet(RCC->CR, RCC_CR_HSIRDY))) {
					frequency = HSI_VALUE;
				}
				// Check if USART2 clock selection is SYSCLK
				else if (srcclk == RCC_USART2CLKSOURCE_SYSCLK) {
					frequency = RCC_GetSysClockFreq();
				}
				// Check if LSE is ready  and if USART2 clock selection is LSE
				else if ((srcclk == RCC_USART2CLKSOURCE_LSE)
					&& (isBitSet(RCC->CSR, RCC_CSR_LSERDY))) {
					frequency = LSE_VALUE;
				}
				// Clock not enabled for USART2
				else {
					frequency = 0U;
				}
				break;
			}
			case RCC_PERIPHCLK_LPUART1: {
				// Get the current LPUART1 source
				srcclk = RCC_GET_LPUART1_SOURCE();

				// Check if LPUART1 clock selection is PCLK1
				if (srcclk == RCC_LPUART1CLKSOURCE_PCLK1) {
					frequency = RCC_GetPCLK1Freq();
				}
				// Check if HSI is ready and if LPUART1 clock selection is HSI
				else if ((srcclk == RCC_LPUART1CLKSOURCE_HSI)
					&& (isBitSet(RCC->CR, RCC_CR_HSIRDY))) {
					frequency = HSI_VALUE;
				}
				// Check if LPUART1 clock selection is SYSCLK
				else if (srcclk == RCC_LPUART1CLKSOURCE_SYSCLK) {
					frequency = RCC_GetSysClockFreq();
				}
				// Check if LSE is ready  and if LPUART1 clock selection is LSE
				else if ((srcclk == RCC_LPUART1CLKSOURCE_LSE)
					&& (isBitSet(RCC->CSR, RCC_CSR_LSERDY))) {
					frequency = LSE_VALUE;
				}
				// Clock not enabled for LPUART1
				else {
					frequency = 0U;
				}
				break;
			}
			default: {
				break;
			}
		}
		return (frequency);
	}
}
