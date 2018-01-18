#pragma once

/*
 * rcc.h
 *
 *  Created on: 17. dets 2017
 *      Author: Den
 */

#include <stdint.h>

namespace Rcc {
	//
	// Public definitions and constants
	//

	//
	// @brief  RCC PLL configuration structure definition
	//
	typedef struct
	{
		uint32_t PLLState;      //!< PLLState: The new state of the PLL.
								//	This parameter can be a value of @ref RCC_PLL_Config

		uint32_t PLLSource;     //!< PLLSource: PLL entry clock source.
								//	This parameter must be a value of @ref RCC_PLL_Clock_Source

		uint32_t PLLMUL;        //!< PLLMUL: Multiplication factor for PLL VCO input clock
								//	This parameter must be a value of @ref RCC_PLL_Multiplication_Factor

		uint32_t PLLDIV;        //!< PLLDIV: Division factor for PLL VCO input clock
								//	This parameter must be a value of @ref RCC_PLL_Division_Factor
	} RCC_PLLInitTypeDef;

	//
	// @brief  RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
	//
	typedef struct
	{
		uint32_t OscillatorType;        //!< The oscillators to be configured.
										//	 This parameter can be a value of @ref RCC_Oscillator_Type

		uint32_t HSEState;              //!< The new state of the HSE.
										//	 This parameter can be a value of @ref RCC_HSE_Config

		uint32_t LSEState;              //!< The new state of the LSE.
										//	 This parameter can be a value of @ref RCC_LSE_Config

		uint32_t HSIState;              //!< The new state of the HSI.
										//	 This parameter can be a value of @ref RCC_HSI_Config

		uint32_t HSICalibrationValue;   //!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
										//	 This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F

		uint32_t LSIState;              //!< The new state of the LSI.
										//	 This parameter can be a value of @ref RCC_LSI_Config

		uint32_t HSI48State;            //!< The new state of the HSI48.
										//	 This parameter can be a value of @ref RCC_HSI48_Config

		uint32_t MSIState;              //!< The new state of the MSI.
										//	 This parameter can be a value of @ref RCC_MSI_Config

		uint32_t MSICalibrationValue;   //!< The MSI calibration trimming value. (default is RCC_MSICALIBRATION_DEFAULT).
										//	 This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF

		uint32_t MSIClockRange;         //!< The MSI  frequency  range.
										//	  This parameter can be a value of @ref RCC_MSI_Clock_Range

		RCC_PLLInitTypeDef PLL;         //!< PLL structure parameters

	} RCC_OscInitTypeDef;

	#define RCC_PERIPHCLK_USART2           0x00000002U

	#define RCC_LPUART1CLKSOURCE_PCLK1        (0x00000000U)
	#define RCC_LPUART1CLKSOURCE_SYSCLK       RCC_CCIPR_LPUART1SEL_0
	#define RCC_LPUART1CLKSOURCE_HSI          RCC_CCIPR_LPUART1SEL_1
	#define RCC_LPUART1CLKSOURCE_LSE          (RCC_CCIPR_LPUART1SEL_0 | RCC_CCIPR_LPUART1SEL_1)

	#define RCC_USART1CLKSOURCE_PCLK2        (0x00000000U)
	#define RCC_USART1CLKSOURCE_SYSCLK       RCC_CCIPR_USART1SEL_0
	#define RCC_USART1CLKSOURCE_HSI          RCC_CCIPR_USART1SEL_1
	#define RCC_USART1CLKSOURCE_LSE          (RCC_CCIPR_USART1SEL_0 | RCC_CCIPR_USART1SEL_1)

	#define RCC_USART2CLKSOURCE_PCLK1        (0x00000000U)
	#define RCC_USART2CLKSOURCE_SYSCLK       RCC_CCIPR_USART2SEL_0
	#define RCC_USART2CLKSOURCE_HSI          RCC_CCIPR_USART2SEL_1
	#define RCC_USART2CLKSOURCE_LSE          (RCC_CCIPR_USART2SEL_0 | RCC_CCIPR_USART2SEL_1)

	#define  TICK_INT_PRIORITY            (0U)    //!< tick interrupt priority

	//
	// @brief External Low Speed oscillator (LSE) value.
	//        This value is used by the UART, RTC HAL module to compute the system frequency
	//
	#define LSE_VALUE    32768U //!< Value of the External oscillator in Hz

	//
	// @brief Internal High Speed oscillator (HSI) value.
	//        This value is used by the RCC HAL module to compute the system frequency
	//        (when HSI is used as system clock source, directly or through the PLL).
	//
	#define HSI_VALUE    16000000U //!< Value of the Internal oscillator in Hz

	// Defines used for Flags
	constexpr uint8_t CR_REG_INDEX = 1U;
	constexpr uint8_t CSR_REG_INDEX = 2U;
	constexpr uint8_t CRRCR_REG_INDEX = 3U;

	// @defgroup RCC_Flag Flags
	//        Elements values convention: XXXYYYYYb
	//           - YYYYY  : Flag position in the register
	//           - XXX  : Register index
	//                 - 001: CR register
	//                 - 010: CSR register
	//                 - 011: CRRCR register (*)
	// (*)   Applicable only for STM32L052xx, STM32L053xx, (...), STM32L073xx & STM32L082xx
	//
	// Flags in the CR register
	constexpr uint8_t RCC_FLAG_HSIRDY 	= (CR_REG_INDEX << 5) | 2U;     //!< Internal High Speed clock ready flag
	constexpr uint8_t RCC_FLAG_HSIDIV 	= (CR_REG_INDEX << 5) | 4U;     //!< HSI16 divider flag
	constexpr uint8_t RCC_FLAG_MSIRDY 	= (CR_REG_INDEX << 5) | 9U;     //!< MSI clock ready flag
	constexpr uint8_t RCC_FLAG_HSERDY 	= (CR_REG_INDEX << 5) | 17U;    //!< External High Speed clock ready flag
	constexpr uint8_t RCC_FLAG_PLLRDY 	= (CR_REG_INDEX << 5) | 25U;    //!< PLL clock ready flag
	// Flags in the CSR register
	constexpr uint8_t RCC_FLAG_LSIRDY 	= (CSR_REG_INDEX << 5) | 1U;    //!< Internal Low Speed oscillator Ready
	constexpr uint8_t RCC_FLAG_LSERDY 	= (CSR_REG_INDEX << 5) | 9U; 	//!< External Low Speed oscillator Ready
	constexpr uint8_t RCC_FLAG_LSECSS 	= (CSR_REG_INDEX << 5) | 14U;   //!< CSS on LSE failure Detection
	constexpr uint8_t RCC_FLAG_OBLRST 	= (CSR_REG_INDEX << 5) | 25U;   //!< Options bytes loading reset flag
	constexpr uint8_t RCC_FLAG_PINRST 	= (CSR_REG_INDEX << 5) | 26U;   //!< PIN reset flag
	constexpr uint8_t RCC_FLAG_PORRST 	= (CSR_REG_INDEX << 5) | 27U;   //!< POR/PDR reset flag
	constexpr uint8_t RCC_FLAG_SFTRST 	= (CSR_REG_INDEX << 5) | 28U;   //!< Software Reset flag
	constexpr uint8_t RCC_FLAG_IWDGRST 	= (CSR_REG_INDEX << 5) | 29U;   //!< Independent Watchdog reset flag
	constexpr uint8_t RCC_FLAG_WWDGRST 	= (CSR_REG_INDEX << 5) | 30U;   //!< Window watchdog reset flag
	constexpr uint8_t RCC_FLAG_LPWRRST	= (CSR_REG_INDEX << 5) | 31U;   //!< Low-Power reset flag
	constexpr uint8_t RCC_FLAG_FWRST   	= (CSR_REG_INDEX << 5) |  8U;   //!< RCC flag FW reset
	// Flags in the CRRCR register
	constexpr uint8_t RCC_FLAG_HSI48RDY = (CRRCR_REG_INDEX << 5) | 1U;	//!< HSI48 clock ready flag

	#define RCC_HSI_OFF                      0x00000000U   			          //!< HSI clock deactivation
	#define RCC_HSI_ON                       RCC_CR_HSION                     //!< HSI clock activation
	#define RCC_HSI_DIV4                     (RCC_CR_HSIDIVEN | RCC_CR_HSION) //!< HSI_DIV4 clock activation

	// @defgroup RCC_Oscillator_Type Oscillator Type
	//
	#define RCC_OSCILLATORTYPE_NONE            0x00000000U
	#define RCC_OSCILLATORTYPE_HSE             0x00000001U
	#define RCC_OSCILLATORTYPE_HSI             0x00000002U
	#define RCC_OSCILLATORTYPE_LSE             0x00000004U
	#define RCC_OSCILLATORTYPE_LSI             0x00000008U
	#define RCC_OSCILLATORTYPE_MSI             0x00000010U
	#define RCC_OSCILLATORTYPE_HSI48           0x00000020U

	#define RCC_SYSCFG_CLK_ENABLE()   SET_BIT(RCC->APB2ENR, (RCC_APB2ENR_SYSCFGEN))

	#define RCC_LPUART1_CLK_ENABLE() SET_BIT(RCC->APB1ENR, (RCC_APB1ENR_LPUART1EN))
	#define RCC_USART2_CLK_ENABLE()  SET_BIT(RCC->APB1ENR, (RCC_APB1ENR_USART2EN))
	#define RCC_LPUART1_CLK_DISABLE() CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_LPUART1EN))
	#define RCC_USART2_CLK_DISABLE()  CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_USART2EN))

	// @brief  Macro to get the LPUART1 clock source.
	// @retval The clock source can be one of the following values:
	//            @arg @ref RCC_LPUART1CLKSOURCE_PCLK1 PCLK1 selected as LPUART1 clock
	//            @arg @ref RCC_LPUART1CLKSOURCE_HSI HSI selected as LPUART1 clock
	//            @arg @ref RCC_LPUART1CLKSOURCE_SYSCLK System Clock selected as LPUART1 clock
	//            @arg @ref RCC_LPUART1CLKSOURCE_LSE LSE selected as LPUART1 clock
	//
	#define RCC_GET_LPUART1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_LPUART1SEL)))

	// @brief  Macro to get the USART1 clock source.
	// @retval The clock source can be one of the following values:
	//            @arg @ref RCC_USART1CLKSOURCE_PCLK2 PCLK2 selected as USART1 clock
	//            @arg @ref RCC_USART1CLKSOURCE_HSI HSI selected as USART1 clock
	//            @arg @ref RCC_USART1CLKSOURCE_SYSCLK System Clock selected as USART1 clock
	//            @arg @ref RCC_USART1CLKSOURCE_LSE LSE selected as USART1 clock
	//
	#define RCC_GET_USART1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_USART1SEL)))

	// @brief  Macro to get the USART2 clock source.
	// @retval The clock source can be one of the following values:
	//            @arg @ref RCC_USART2CLKSOURCE_PCLK1 PCLK1 selected as USART2 clock
	//            @arg @ref RCC_USART2CLKSOURCE_HSI HSI selected as USART2 clock
	//            @arg @ref RCC_USART2CLKSOURCE_SYSCLK System Clock selected as USART2 clock
	//            @arg @ref RCC_USART2CLKSOURCE_LSE LSE selected as USART2 clock
	//
	#define RCC_GET_USART2_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_USART2SEL)))

	// @brief  Macro to get the USART1 clock source.
	// @retval The clock source can be one of the following values:
	//            @arg @ref RCC_USART1CLKSOURCE_PCLK2 PCLK2 selected as USART1 clock
	//            @arg @ref RCC_USART1CLKSOURCE_HSI HSI selected as USART1 clock
	//            @arg @ref RCC_USART1CLKSOURCE_SYSCLK System Clock selected as USART1 clock
	//            @arg @ref RCC_USART1CLKSOURCE_LSE LSE selected as USART1 clock
	//
	#define RCC_GET_USART1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_USART1SEL)))

	// @brief  Macro to get the USART2 clock source.
	// @retval The clock source can be one of the following values:
	//            @arg @ref RCC_USART2CLKSOURCE_PCLK1 PCLK1 selected as USART2 clock
	//            @arg @ref RCC_USART2CLKSOURCE_HSI HSI selected as USART2 clock
	//            @arg @ref RCC_USART2CLKSOURCE_SYSCLK System Clock selected as USART2 clock
	//            @arg @ref RCC_USART2CLKSOURCE_LSE LSE selected as USART2 clock
	//
	#define RCC_GET_USART2_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_USART2SEL)))

	// @brief  Macro to get the LPUART1 clock source.
	// @retval The clock source can be one of the following values:
	//            @arg @ref RCC_LPUART1CLKSOURCE_PCLK1 PCLK1 selected as LPUART1 clock
	//            @arg @ref RCC_LPUART1CLKSOURCE_HSI HSI selected as LPUART1 clock
	//            @arg @ref RCC_LPUART1CLKSOURCE_SYSCLK System Clock selected as LPUART1 clock
	//            @arg @ref RCC_LPUART1CLKSOURCE_LSE LSE selected as LPUART1 clock
	//
	#define RCC_GET_LPUART1_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_LPUART1SEL)))

	// @defgroup RCC_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
	// @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
	// @note   After reset, the peripheral clock (used for registers read/write access)
	//         is disabled and the application software has to enable this clock before
	//         using it.
	//
	#define RCC_PWR_CLK_ENABLE()     SET_BIT(RCC->APB1ENR, (RCC_APB1ENR_PWREN))

	//
	// Public variables
	//


	//
	// Public functions
	//

	// @brief  Check RCC flag is set or not.
	// @param  __FLAG__ specifies the flag to check.
	//         This parameter can be one of the following values:
	//     @arg @ref RCC_FLAG_HSIRDY HSI oscillator clock ready
	//     @arg @ref RCC_FLAG_HSI48RDY HSI48 oscillator clock ready (not available on all devices)
	//     @arg @ref RCC_FLAG_HSIDIV HSI16 divider flag
	//     @arg @ref RCC_FLAG_MSIRDY MSI oscillator clock ready
	//     @arg @ref RCC_FLAG_HSERDY HSE oscillator clock ready
	//     @arg @ref RCC_FLAG_PLLRDY PLL clock ready
	//     @arg @ref RCC_FLAG_LSECSS LSE oscillator clock CSS detected
	//     @arg @ref RCC_FLAG_LSERDY LSE oscillator clock ready
	//     @arg @ref RCC_FLAG_FWRST Firewall reset
	//     @arg @ref RCC_FLAG_LSIRDY LSI oscillator clock ready
	//     @arg @ref RCC_FLAG_OBLRST Option Byte Loader (OBL) reset
	//     @arg @ref RCC_FLAG_PINRST Pin reset
	//     @arg @ref RCC_FLAG_PORRST POR/PDR reset
	//     @arg @ref RCC_FLAG_SFTRST Software reset
	//     @arg @ref RCC_FLAG_IWDGRST Independent Watchdog reset
	//     @arg @ref RCC_FLAG_WWDGRST Window Watchdog reset
	//     @arg @ref RCC_FLAG_LPWRRST Low Power reset
	// @retval The new state of __FLAG__ (TRUE or FALSE).
	//
	uint8_t RCC_GetFlag(uint8_t flag);

	// System Clock Configuration
	//
	void SystemClock_Config(void);

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
	uint32_t RCC_GetSysClockFreq(void);

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
	StatusTypeDef RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);

	void RCC_ClearFlag(void);

	//
	// @brief  Returns the PCLK1 frequency
	// @note   Each time PCLK1 changes, this function must be called to update the
	//         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
	// @retval PCLK1 frequency
	//
	uint32_t RCC_GetPCLK1Freq(void);

	//
	// @brief  Returns the PCLK2 frequency
	// @note   Each time PCLK2 changes, this function must be called to update the
	//         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
	// @retval PCLK2 frequency
	//
	uint32_t RCC_GetPCLK2Freq(void);

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
	uint32_t RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);

	//
	// @brief  Configures the RCC_OscInitStruct according to the internal
	// RCC configuration registers.
	// @param  RCC_OscInitStruct pointer to an RCC_OscInitTypeDef structure that
	// will be configured.
	// @retval None
	//
	void RCC_GetOscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);

	void RCC_GPIOA_CLK_ENABLE();
	void RCC_GPIOB_CLK_ENABLE();
	void RCC_GPIOC_CLK_ENABLE();
	void RCC_GPIOH_CLK_ENABLE();

	// End

}
