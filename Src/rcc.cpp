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
#include "Timers.h"
#include "Thermoprinter.h"

namespace Rcc {
	//
	// Private definitions and constants
	//

	//
	//@brief  RCC extended clocks structure definition
	//
	typedef struct
	{
		uint32_t PeriphClockSelection;  //!< The Extended Clock to be configured.
										//	This parameter can be a value of @ref RCCEx_Periph_Clock_Selection

		uint32_t RTCClockSelection;     //!< specifies the RTC clock source.
										//	 This parameter can be a value of @ref RCC_RTC_LCD_Clock_Source

		uint32_t LCDClockSelection;     //!< specifies the LCD clock source.
										//	 This parameter can be a value of @ref RCC_RTC_LCD_Clock_Source

		uint32_t Usart1ClockSelection;  //!< USART1 clock source
										//	  This parameter can be a value of @ref RCCEx_USART1_Clock_Source

		uint32_t Usart2ClockSelection;  //!< USART2 clock source
										//	  This parameter can be a value of @ref RCCEx_USART2_Clock_Source

		uint32_t Lpuart1ClockSelection; //!< LPUART1 clock source
										//	  This parameter can be a value of @ref RCCEx_LPUART1_Clock_Source

		uint32_t I2c1ClockSelection;    //!< I2C1 clock source
										//	  This parameter can be a value of @ref RCCEx_I2C1_Clock_Source

		uint32_t LptimClockSelection;   //!< LPTIM1 clock source
										//	  This parameter can be a value of @ref RCCEx_LPTIM1_Clock_Source
		uint32_t UsbClockSelection;     //!< Specifies USB and RNG Clock  Selection
										//	  This parameter can be a value of @ref RCCEx_USB_Clock_Source
	} RCC_PeriphCLKInitTypeDef;

	//
	// @brief  RCC System, AHB and APB busses clock configuration structure definition
	//
	typedef struct
	{
		uint32_t ClockType;             //!< The clock to be configured.
										//	 This parameter can be a value of @ref RCC_System_Clock_Type

		uint32_t SYSCLKSource;          //!< The clock source (SYSCLKS) used as system clock.
										//	 This parameter can be a value of @ref RCC_System_Clock_Source

		uint32_t AHBCLKDivider;         //!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
										//	 This parameter can be a value of @ref RCC_AHB_Clock_Source

		uint32_t APB1CLKDivider;        //!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
										//	 This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source

		uint32_t APB2CLKDivider;        //!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
										//	 This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source
	} RCC_ClkInitTypeDef;

	// @defgroup CORTEX_SysTick_clock_source CORTEX SysTick Clock Source
	//
	#define SYSTICK_CLKSOURCE_HCLK_DIV8    0x00000000U
	#define SYSTICK_CLKSOURCE_HCLK         0x00000004U

	// @defgroup RCCEx_USB_Clock_Source RCCEx USB Clock Source
	//
	#define RCC_USBCLKSOURCE_HSI48           RCC_CCIPR_HSI48SEL
	#define RCC_USBCLKSOURCE_PLL             0x00000000U

	#define RCC_PERIPHCLK_USART1           0x00000001U
	#define RCC_PERIPHCLK_USART2           0x00000002U
	#define RCC_PERIPHCLK_LPUART1          0x00000004U
	#define RCC_PERIPHCLK_I2C1             0x00000008U
	#define RCC_PERIPHCLK_I2C2             0x00000010U
	#define RCC_PERIPHCLK_RTC              0x00000020U
	#define RCC_PERIPHCLK_USB              0x00000040U
	#define RCC_PERIPHCLK_LPTIM1           0x00000080U
	#define RCC_PERIPHCLK_LCD              0x00000800U

	// @defgroup RCC_APB1_APB2_Clock_Source APB1 APB2 Clock Source
	//
	#define RCC_HCLK_DIV1                    RCC_CFGR_PPRE1_DIV1  //!< HCLK not divided
	#define RCC_HCLK_DIV2                    RCC_CFGR_PPRE1_DIV2  //!< HCLK divided by 2
	#define RCC_HCLK_DIV4                    RCC_CFGR_PPRE1_DIV4  //!< HCLK divided by 4
	#define RCC_HCLK_DIV8                    RCC_CFGR_PPRE1_DIV8  //!< HCLK divided by 8
	#define RCC_HCLK_DIV16                   RCC_CFGR_PPRE1_DIV16 //!< HCLK divided by 16

	#define RCC_CLOCKTYPE_SYSCLK             0x00000001U //!< SYSCLK to configure
	#define RCC_CLOCKTYPE_HCLK               0x00000002U //!< HCLK to configure
	#define RCC_CLOCKTYPE_PCLK1              0x00000004U //!< PCLK1 to configure
	#define RCC_CLOCKTYPE_PCLK2              0x00000008U //!< PCLK2 to configure

	#define RCC_SYSCLKSOURCE_MSI             RCC_CFGR_SW_MSI //!< MSI selected as system clock
	#define RCC_SYSCLKSOURCE_HSI             RCC_CFGR_SW_HSI //!< HSI selected as system clock
	#define RCC_SYSCLKSOURCE_HSE             RCC_CFGR_SW_HSE //!< HSE selected as system clock
	#define RCC_SYSCLKSOURCE_PLLCLK          RCC_CFGR_SW_PLL //!< PLL selected as system clock

	// @defgroup RCC_HSI48_Config HSI48 Config
	//
	#define RCC_HSI48_OFF               0x00U
	#define RCC_HSI48_ON                0x01U

	// @defgroup RCC_LSE_Config LSE Config
	//
	#define RCC_LSE_OFF                      0x00000000U                       /*!< LSE clock deactivation */
	#define RCC_LSE_ON                       RCC_CSR_LSEON                                /*!< LSE clock activation */
	#define RCC_LSE_BYPASS                   ((uint32_t)(RCC_CSR_LSEBYP | RCC_CSR_LSEON)) /*!< External clock source for LSE clock */

	// @defgroup RCC_LSI_Config LSI Config
	//
	#define RCC_LSI_OFF                      0x00000000U   /*!< LSI clock deactivation */
	#define RCC_LSI_ON                       RCC_CSR_LSION            /*!< LSI clock activation */

	// @defgroup RCC_MSI_Clock_Range MSI Clock Range
	//
	#define RCC_MSIRANGE_0                   RCC_ICSCR_MSIRANGE_0 //!< MSI = 65.536 KHz
	#define RCC_MSIRANGE_1                   RCC_ICSCR_MSIRANGE_1 //!< MSI = 131.072 KHz
	#define RCC_MSIRANGE_2                   RCC_ICSCR_MSIRANGE_2 //!< MSI = 262.144 KHz
	#define RCC_MSIRANGE_3                   RCC_ICSCR_MSIRANGE_3 //!< MSI = 524.288 KHz
	#define RCC_MSIRANGE_4                   RCC_ICSCR_MSIRANGE_4 //!< MSI = 1.048 MHz
	#define RCC_MSIRANGE_5                   RCC_ICSCR_MSIRANGE_5 //!< MSI = 2.097 MHz
	#define RCC_MSIRANGE_6                   RCC_ICSCR_MSIRANGE_6 //!< MSI = 4.194 MHz

	// @defgroup RCC_MSI_Config MSI Config
	//
	#define RCC_MSI_OFF                      ((uint32_t)0x00000000)
	#define RCC_MSI_ON                       ((uint32_t)0x00000001)

	// @defgroup RCC_HSE_Config HSE Config
	//
	#define RCC_HSE_OFF                      ((uint32_t)0x00000000)                     /*!< HSE clock deactivation */
	#define RCC_HSE_ON                       RCC_CR_HSEON                               /*!< HSE clock activation */
	#define RCC_HSE_BYPASS                   ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON)) /*!< External clock source for HSE clock */

	#define PWR_REGULATOR_VOLTAGE_SCALE1   PWR_CR_VOS_0
	#define PWR_REGULATOR_VOLTAGE_SCALE2   PWR_CR_VOS_1
	#define PWR_REGULATOR_VOLTAGE_SCALE3   PWR_CR_VOS

	// @defgroup RCC_AHB_Clock_Source AHB Clock Source
	//
	#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1   //!< SYSCLK not divided
	#define RCC_SYSCLK_DIV2                  RCC_CFGR_HPRE_DIV2   //!< SYSCLK divided by 2
	#define RCC_SYSCLK_DIV4                  RCC_CFGR_HPRE_DIV4   //!< SYSCLK divided by 4
	#define RCC_SYSCLK_DIV8                  RCC_CFGR_HPRE_DIV8   //!< SYSCLK divided by 8
	#define RCC_SYSCLK_DIV16                 RCC_CFGR_HPRE_DIV16  //!< SYSCLK divided by 16
	#define RCC_SYSCLK_DIV64                 RCC_CFGR_HPRE_DIV64  //!< SYSCLK divided by 64
	#define RCC_SYSCLK_DIV128                RCC_CFGR_HPRE_DIV128 //!< SYSCLK divided by 128
	#define RCC_SYSCLK_DIV256                RCC_CFGR_HPRE_DIV256 //!< SYSCLK divided by 256
	#define RCC_SYSCLK_DIV512                RCC_CFGR_HPRE_DIV512 //!< SYSCLK divided by 512

	// @defgroup FLASH_Latency FLASH Latency
	//
	#define FLASH_LATENCY_0            (0x00000000U)    /*!< FLASH Zero Latency cycle */
	#define FLASH_LATENCY_1            FLASH_ACR_LATENCY         /*!< FLASH One Latency cycle */

	// @defgroup RCC_Timeout RCC Timeout
	//
	// Disable Backup domain write protection state change timeout
	#define RCC_DBP_TIMEOUT_VALUE      (100U)       // 100 ms
	// LSE state change timeout
	#define RCC_LSE_TIMEOUT_VALUE      LSE_STARTUP_TIMEOUT
	#define CLOCKSWITCH_TIMEOUT_VALUE  (5000U)  // 5 s
	#define HSE_TIMEOUT_VALUE          HSE_STARTUP_TIMEOUT
	#define MSI_TIMEOUT_VALUE          (2U)      // 2 ms (minimum Tick + 1)
	#define HSI_TIMEOUT_VALUE          (2U)      // 2 ms (minimum Tick + 1)
	#define HSI48_TIMEOUT_VALUE        (2U)      // 2 ms (minimum Tick + 1)
	#define LSI_TIMEOUT_VALUE          (2U)      // 2 ms (minimum Tick + 1)
	#define PLL_TIMEOUT_VALUE          (2U)      // 2 ms (minimum Tick + 1)
	#define HSI48_TIMEOUT_VALUE        (2U)      // 2 ms (minimum Tick + 1)

	#define LSE_STARTUP_TIMEOUT  5000U   //!< Time out for LSE start up, in ms

	//
	// @brief Internal Low Speed oscillator (LSI) value.
	//
	#define LSI_VALUE  37000U       //!< LSI Typical Value in Hz
	                      	  	    //!< Value of the Internal Low Speed oscillator in Hz
									//			 The real value may vary depending on the variations
									//			 in voltage and temperature.

	// ########################## Oscillator Values adaptation ####################
	//
	// @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
	//        This value is used by the RCC HAL module to compute the system frequency
	//        (when HSE is used as system clock source, directly or through the PLL).
	//
	#define HSE_VALUE    8000000U //!< Value of the External oscillator in Hz

	#define HSE_STARTUP_TIMEOUT    100U   //!< Time out for HSE start up, in ms

	//
	// @brief Internal Multiple Speed oscillator (MSI) default value.
	//        This value is the default MSI range value after Reset.
	//
	#define MSI_VALUE    2097000U //!< Value of the Internal oscillator in Hz

	//
	// @brief Internal High Speed oscillator for USB (HSI48) value.
	//
	#define HSI48_VALUE 48000000U   //!< Value of the Internal High Speed oscillator for USB in Hz.
									//			 The real value may vary depending on the variations
									//			 in voltage and temperature.

	constexpr uint8_t RCC_FLAG_MASK = 0x1FU;

	#define RCC_PLLSOURCE_HSI           RCC_CFGR_PLLSRC_HSI        //!< HSI clock selected as PLL entry clock source
	#define RCC_PLLSOURCE_HSE           RCC_CFGR_PLLSRC_HSE        //!< HSE clock selected as PLL entry clock source

	#define RCC_SYSCLKSOURCE_STATUS_MSI      RCC_CFGR_SWS_MSI            //!< MSI used as system clock
	#define RCC_SYSCLKSOURCE_STATUS_HSI      RCC_CFGR_SWS_HSI            //!< HSI used as system clock
	#define RCC_SYSCLKSOURCE_STATUS_HSE      RCC_CFGR_SWS_HSE            //!< HSE used as system clock
	#define RCC_SYSCLKSOURCE_STATUS_PLLCLK   RCC_CFGR_SWS_PLL            //!< PLL used as system clock

	#define RCC_PLL_NONE                      0x00000000U  //!< PLL is not configured
	#define RCC_PLL_OFF                       0x00000001U  //!< PLL deactivation
	#define RCC_PLL_ON                        0x00000002U  //!< PLL activation

	// Bits position in  in the CFGR register
	#define RCC_CFGR_PLLMUL_BITNUMBER         RCC_CFGR_PLLMUL_Pos
	#define RCC_CFGR_PLLDIV_BITNUMBER         RCC_CFGR_PLLDIV_Pos
	#define RCC_CFGR_HPRE_BITNUMBER           RCC_CFGR_HPRE_Pos
	#define RCC_CFGR_PPRE1_BITNUMBER          RCC_CFGR_PPRE1_Pos
	#define RCC_CFGR_PPRE2_BITNUMBER          RCC_CFGR_PPRE2_Pos
	// Bits position in  in the ICSCR register
	#define RCC_ICSCR_MSIRANGE_BITNUMBER      RCC_ICSCR_MSIRANGE_Pos
	#define RCC_ICSCR_MSITRIM_BITNUMBER       RCC_ICSCR_MSITRIM_Pos

	// @brief  Macro to get the Internal 48Mhz High Speed oscillator (HSI48) state.
	// @retval The clock source can be one of the following values:
	//            @arg @ref RCC_HSI48_ON  HSI48 enabled
	//            @arg @ref RCC_HSI48_OFF HSI48 disabled
	//
	#define RCC_GET_HSI48_STATE() \
					  (((uint32_t)(READ_BIT(RCC->CRRCR, RCC_CRRCR_HSI48ON)) != RESET) ? RCC_HSI48_ON : RCC_HSI48_OFF)

	// @brief  Macro to get the USB clock source.
	// @retval The clock source can be one of the following values:
	//            @arg @ref RCC_USBCLKSOURCE_HSI48  HSI48 selected as USB clock
	//            @arg @ref RCC_USBCLKSOURCE_PLL PLL Clock selected as USB clock
	//
	#define RCC_GET_USB_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_HSI48SEL)))

	// @brief  Macro to configure the USB clock (USBCLK).
	// @param  __USB_CLKSOURCE__ specifies the USB clock source.
	//         This parameter can be one of the following values:
	//            @arg @ref RCC_USBCLKSOURCE_HSI48  HSI48 selected as USB clock
	//            @arg @ref RCC_USBCLKSOURCE_PLL PLL Clock selected as USB clock
	//
	#define RCC_USB_CONFIG(__USB_CLKSOURCE__) \
					  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_HSI48SEL, (uint32_t)(__USB_CLKSOURCE__))

	// @brief  Macro to get the USB clock source.
	// @retval The clock source can be one of the following values:
	//            @arg @ref RCC_USBCLKSOURCE_HSI48  HSI48 selected as USB clock
	//            @arg @ref RCC_USBCLKSOURCE_PLL PLL Clock selected as USB clock
	//
	#define RCC_GET_USB_SOURCE() ((uint32_t)(READ_BIT(RCC->CCIPR, RCC_CCIPR_HSI48SEL)))

	// @brief Macro to configure the I2C1 clock (I2C1CLK).
	//
	// @param  __I2C1_CLKSOURCE__ specifies the I2C1 clock source.
	//          This parameter can be one of the following values:
	//            @arg @ref RCC_I2C1CLKSOURCE_PCLK1 PCLK1 selected as I2C1 clock
	//            @arg @ref RCC_I2C1CLKSOURCE_HSI HSI selected as I2C1 clock
	//            @arg @ref RCC_I2C1CLKSOURCE_SYSCLK System Clock selected as I2C1 clock
	//
	#define RCC_I2C1_CONFIG(__I2C1_CLKSOURCE__) \
					  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_I2C1SEL, (uint32_t)(__I2C1_CLKSOURCE__))

	// @brief Macro to configure the LPTIM1 clock (LPTIM1CLK).
	//
	// @param  __LPTIM1_CLKSOURCE__ specifies the LPTIM1 clock source.
	//          This parameter can be one of the following values:
	//            @arg @ref RCC_LPTIM1CLKSOURCE_PCLK PCLK selected as LPTIM1 clock
	//            @arg @ref RCC_LPTIM1CLKSOURCE_LSI  HSI  selected as LPTIM1 clock
	//            @arg @ref RCC_LPTIM1CLKSOURCE_HSI  LSI  selected as LPTIM1 clock
	//            @arg @ref RCC_LPTIM1CLKSOURCE_LSE  LSE  selected as LPTIM1 clock
	//
	#define RCC_LPTIM1_CONFIG(__LPTIM1_CLKSOURCE__) \
					  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPTIM1SEL, (uint32_t)(__LPTIM1_CLKSOURCE__))

	// @brief Macro to configure the LPUART1 clock (LPUART1CLK).
	//
	// @param  __LPUART1_CLKSOURCE__ specifies the LPUART1 clock source.
	//          This parameter can be one of the following values:
	//            @arg @ref RCC_LPUART1CLKSOURCE_PCLK1 PCLK1 selected as LPUART1 clock
	//            @arg @ref RCC_LPUART1CLKSOURCE_HSI HSI selected as LPUART1 clock
	//            @arg @ref RCC_LPUART1CLKSOURCE_SYSCLK System Clock selected as LPUART1 clock
	//            @arg @ref RCC_LPUART1CLKSOURCE_LSE LSE selected as LPUART1 clock
	//
	#define RCC_LPUART1_CONFIG(__LPUART1_CLKSOURCE__) \
					  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_LPUART1SEL, (uint32_t)(__LPUART1_CLKSOURCE__))

	// @brief Macro to configure the USART2 clock (USART2CLK).
	//
	// @param  __USART2_CLKSOURCE__ specifies the USART2 clock source.
	//          This parameter can be one of the following values:
	//            @arg @ref RCC_USART2CLKSOURCE_PCLK1 PCLK1 selected as USART2 clock
	//            @arg @ref RCC_USART2CLKSOURCE_HSI HSI selected as USART2 clock
	//            @arg @ref RCC_USART2CLKSOURCE_SYSCLK System Clock selected as USART2 clock
	//            @arg @ref RCC_USART2CLKSOURCE_LSE LSE selected as USART2 clock
	//
	#define RCC_USART2_CONFIG(__USART2_CLKSOURCE__) \
					  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_USART2SEL, (uint32_t)(__USART2_CLKSOURCE__))

	#define RCC_RTC_CONFIG(__RTC_CLKSOURCE__) do { \
										  RCC_RTC_CLKPRESCALER(__RTC_CLKSOURCE__);      \
										  RCC->CSR |= ((__RTC_CLKSOURCE__) & RCC_CSR_RTCSEL); \
										} while (0)

	// @brief Macro to configure the USART1 clock (USART1CLK).
	//
	// @param  __USART1_CLKSOURCE__ specifies the USART1 clock source.
	//          This parameter can be one of the following values:
	//            @arg @ref RCC_USART1CLKSOURCE_PCLK2 PCLK2 selected as USART1 clock
	//            @arg @ref RCC_USART1CLKSOURCE_HSI HSI selected as USART1 clock
	//            @arg @ref RCC_USART1CLKSOURCE_SYSCLK System Clock selected as USART1 clock
	//            @arg @ref RCC_USART1CLKSOURCE_LSE LSE selected as USART1 clock
	//
	#define RCC_USART1_CONFIG(__USART1_CLKSOURCE__) \
					  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_USART1SEL, (uint32_t)(__USART1_CLKSOURCE__))

	// @brief Macro to configure the RTC clock (RTCCLK).
	// @note   As the RTC clock configuration bits are in the Backup domain and write
	//         access is denied to this domain after reset, you have to enable write
	//         access using the Power Backup Access macro before to configure
	//         the RTC clock source (to be done once after reset).
	// @note   Once the RTC clock is configured it cannot be changed unless the
	//         Backup domain is reset using @ref __HAL_RCC_BACKUPRESET_FORCE() macro, or by
	//         a Power On Reset (POR).
	// @note   RTC prescaler cannot be modified if HSE is enabled (HSEON = 1).
	//
	// @param  __RTC_CLKSOURCE__ specifies the RTC clock source.
	//          This parameter can be one of the following values:
	//             @arg @ref RCC_RTCCLKSOURCE_NO_CLK No clock selected as RTC clock
	//             @arg @ref RCC_RTCCLKSOURCE_LSE LSE selected as RTC clock
	//             @arg @ref RCC_RTCCLKSOURCE_LSI LSI selected as RTC clock
	//             @arg @ref RCC_RTCCLKSOURCE_HSE_DIV2 HSE divided by 2 selected as RTC clock
	//             @arg @ref RCC_RTCCLKSOURCE_HSE_DIV4 HSE divided by 4 selected as RTC clock
	//             @arg @ref RCC_RTCCLKSOURCE_HSE_DIV8 HSE divided by 8 selected as RTC clock
	//             @arg @ref RCC_RTCCLKSOURCE_HSE_DIV16 HSE divided by 16 selected as RTC clock
	// @note   If the LSE or LSI is used as RTC clock source, the RTC continues to
	//         work in STOP and STANDBY modes, and can be used as wakeup source.
	//         However, when the HSE clock is used as RTC clock source, the RTC
	//         cannot be used in STOP and STANDBY modes.
	// @note   The maximum input clock frequency for RTC is 1MHz (when using HSE as
	//         RTC clock source).
	//
	#define RCC_RTC_CLKPRESCALER(__RTC_CLKSOURCE__) do { \
				if(((__RTC_CLKSOURCE__) & RCC_CSR_RTCSEL_HSE) == RCC_CSR_RTCSEL_HSE)          \
				{                                                                             \
				  MODIFY_REG(RCC->CR, RCC_CR_RTCPRE, ((__RTC_CLKSOURCE__) & RCC_CR_RTCPRE));  \
				}                                                                             \
			  } while (0)

	//
	// @brief  Macro to configure the system clock source.
	// @param  __SYSCLKSOURCE__ specifies the system clock source.
	//          This parameter can be one of the following values:
	//              @arg @ref RCC_SYSCLKSOURCE_MSI MSI oscillator is used as system clock source.
	//              @arg @ref RCC_SYSCLKSOURCE_HSI HSI oscillator is used as system clock source.
	//              @arg @ref RCC_SYSCLKSOURCE_HSE HSE oscillator is used as system clock source.
	//              @arg @ref RCC_SYSCLKSOURCE_PLLCLK PLL output is used as system clock source.
	//
	#define RCC_SYSCLK_CONFIG(__SYSCLKSOURCE__) \
					  MODIFY_REG(RCC->CFGR, RCC_CFGR_SW, (__SYSCLKSOURCE__))

	// @brief  Macro to force the Backup domain reset.
	// @note   This function resets the RTC peripheral (including the backup registers)
	//         and the RTC clock source selection in RCC_CSR register.
	// @note   The BKPSRAM is not affected by this reset.
	//
	#define RCC_BACKUPRESET_FORCE()  SET_BIT(RCC->CSR, RCC_CSR_RTCRST)

	// @brief  Macros to release the Backup domain reset.
	//
	#define RCC_BACKUPRESET_RELEASE() CLEAR_BIT(RCC->CSR, RCC_CSR_RTCRST)

	// @brief Macro to disable the main PLL.
	// @note   The main PLL can not be disabled if it is used as system clock source
	//
	#define RCC_PLL_DISABLE() CLEAR_BIT(RCC->CR, RCC_CR_PLLON)

	// @brief Macro to configure the main PLL clock source, multiplication and division factors.
	// @note   This function must be used only when the main PLL is disabled.
	//
	// @param  __RCC_PLLSOURCE__ specifies the PLL entry clock source.
	//          This parameter can be one of the following values:
	//            @arg @ref RCC_PLLSOURCE_HSI HSI oscillator clock selected as PLL clock entry
	//            @arg @ref RCC_PLLSOURCE_HSE HSE oscillator clock selected as PLL clock entry
	// @param  __PLLMUL__ specifies the multiplication factor for PLL VCO output clock
	//          This parameter can be one of the following values:
	//             @arg @ref RCC_PLL_MUL3   PLLVCO = PLL clock entry x 3
	//             @arg @ref RCC_PLL_MUL4   PLLVCO = PLL clock entry x 4
	//             @arg @ref RCC_PLL_MUL6   PLLVCO = PLL clock entry x 6
	//             @arg @ref RCC_PLL_MUL8   PLLVCO = PLL clock entry x 8
	//             @arg @ref RCC_PLL_MUL12  PLLVCO = PLL clock entry x 12
	//             @arg @ref RCC_PLL_MUL16  PLLVCO = PLL clock entry x 16
	//             @arg @ref RCC_PLL_MUL24  PLLVCO = PLL clock entry x 24
	//             @arg @ref RCC_PLL_MUL32  PLLVCO = PLL clock entry x 32
	//             @arg @ref RCC_PLL_MUL48  PLLVCO = PLL clock entry x 48
	// @note The PLL VCO clock frequency must not exceed 96 MHz when the product is in
	//          Range 1, 48 MHz when the product is in Range 2 and 24 MHz when the product is
	//          in Range 3.
	//
	// @param  __PLLDIV__ specifies the division factor for PLL VCO input clock
	//          This parameter can be one of the following values:
	//             @arg @ref RCC_PLL_DIV2 PLL clock output = PLLVCO / 2
	//             @arg @ref RCC_PLL_DIV3 PLL clock output = PLLVCO / 3
	//             @arg @ref RCC_PLL_DIV4 PLL clock output = PLLVCO / 4
	//
	//
	#define RCC_PLL_CONFIG(__RCC_PLLSOURCE__, __PLLMUL__, __PLLDIV__)\
			MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLSRC|RCC_CFGR_PLLMUL|RCC_CFGR_PLLDIV),((__RCC_PLLSOURCE__) | (__PLLMUL__) | (__PLLDIV__)))

	// @brief Macro to enable the main PLL.
	// @note   After enabling the main PLL, the application software should wait on
	//         PLLRDY flag to be set indicating that PLL clock is stable and can
	//         be used as system clock source.
	// @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
	//
	#define RCC_PLL_ENABLE() SET_BIT(RCC->CR, RCC_CR_PLLON)

	//
	// @brief  Enable the Internal High Speed oscillator for USB (HSI48).
	// @note   After enabling the HSI48, the application software should wait on
	//         HSI48RDY flag to be set indicating that HSI48 clock is stable and can
	//         be used to clock the USB.
	// @note   The HSI48 is stopped by hardware when entering STOP and STANDBY modes.
	//
	#define RCC_HSI48_ENABLE()  do { SET_BIT(RCC->CRRCR, RCC_CRRCR_HSI48ON);            \
											 SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);       \
											 SET_BIT(SYSCFG->CFGR3, SYSCFG_CFGR3_ENREF_HSI48);  \
										} while (0)
	//
	// @brief  Disable the Internal High Speed oscillator for USB (HSI48).
	//
	#define RCC_HSI48_DISABLE()  do { CLEAR_BIT(RCC->CRRCR, RCC_CRRCR_HSI48ON);   \
											  CLEAR_BIT(SYSCFG->CFGR3, SYSCFG_CFGR3_ENREF_HSI48);  \
										 } while (0)

	//
	// @brief  Macro to configure the External Low Speed oscillator (LSE).
	// @note Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not supported by this macro.
	// @note   As the LSE is in the Backup domain and write access is denied to
	//         this domain after reset, you have to enable write access using
	//         @ref HAL_PWR_EnableBkUpAccess() function before to configure the LSE
	//         (to be done once after reset).
	// @note   After enabling the LSE (RCC_LSE_ON or RCC_LSE_BYPASS), the application
	//         software should wait on LSERDY flag to be set indicating that LSE clock
	//         is stable and can be used to clock the RTC.
	// @param  __STATE__ specifies the new state of the LSE.
	//         This parameter can be one of the following values:
	//            @arg @ref RCC_LSE_OFF turn OFF the LSE oscillator, LSERDY flag goes low after
	//                              6 LSE oscillator clock cycles.
	//            @arg @ref RCC_LSE_ON turn ON the LSE oscillator.
	//            @arg @ref RCC_LSE_BYPASS LSE oscillator bypassed with external clock.
	//
	#define RCC_LSE_CONFIG(__STATE__)                                     \
					  do{                                                     \
						if ((__STATE__) == RCC_LSE_ON)                        \
						{                                                     \
						  SET_BIT(RCC->CSR, RCC_CSR_LSEON);                   \
						}                                                     \
						else if ((__STATE__) == RCC_LSE_OFF)                  \
						{                                                     \
						  CLEAR_BIT(RCC->CSR, RCC_CSR_LSEON);                 \
						  CLEAR_BIT(RCC->CSR, RCC_CSR_LSEBYP);                \
						}                                                     \
						else if ((__STATE__) == RCC_LSE_BYPASS)               \
						{                                                     \
						  SET_BIT(RCC->CSR, RCC_CSR_LSEBYP);                  \
						  SET_BIT(RCC->CSR, RCC_CSR_LSEON);                   \
						}                                                     \
						else                                                  \
						{                                                     \
						  CLEAR_BIT(RCC->CSR, RCC_CSR_LSEON);                 \
						  CLEAR_BIT(RCC->CSR, RCC_CSR_LSEBYP);                \
						}                                                     \
					  }while(0)

	// @brief Macro to enable the Internal Low Speed oscillator (LSI).
	// @note   After enabling the LSI, the application software should wait on
	//         LSIRDY flag to be set indicating that LSI clock is stable and can
	//         be used to clock the IWDG and/or the RTC.
	//
	#define RCC_LSI_ENABLE() SET_BIT(RCC->CSR, RCC_CSR_LSION)

	// @brief Macro to disable the Internal Low Speed oscillator (LSI).
	// @note   LSI can not be disabled if the IWDG is running.
	// @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
	//         clock cycles.
	//
	#define RCC_LSI_DISABLE() CLEAR_BIT(RCC->CSR, RCC_CSR_LSION)

	// @brief  Macro to configures the Internal Multi Speed oscillator (MSI) clock range.
	// @note     After restart from Reset or wakeup from STANDBY, the MSI clock is
	//           around 2.097 MHz. The MSI clock does not change after wake-up from
	//           STOP mode.
	// @note    The MSI clock range can be modified on the fly.
	// @param  _MSIRANGEVALUE_ specifies the MSI Clock range.
	//   This parameter must be one of the following values:
	//     @arg @ref RCC_MSIRANGE_0 MSI clock is around 65.536 KHz
	//     @arg @ref RCC_MSIRANGE_1 MSI clock is around 131.072 KHz
	//     @arg @ref RCC_MSIRANGE_2 MSI clock is around 262.144 KHz
	//     @arg @ref RCC_MSIRANGE_3 MSI clock is around 524.288 KHz
	//     @arg @ref RCC_MSIRANGE_4 MSI clock is around 1.048 MHz
	//     @arg @ref RCC_MSIRANGE_5 MSI clock is around 2.097 MHz (default after Reset or wake-up from STANDBY)
	//     @arg @ref RCC_MSIRANGE_6 MSI clock is around 4.194 MHz
	//
	#define RCC_MSI_RANGE_CONFIG(_MSIRANGEVALUE_) (MODIFY_REG(RCC->ICSCR, \
			RCC_ICSCR_MSIRANGE, (uint32_t)(_MSIRANGEVALUE_)))

	// @brief  Macro adjusts Internal Multi Speed oscillator (MSI) calibration value.
	// @note   The calibration is used to compensate for the variations in voltage
	//         and temperature that influence the frequency of the internal MSI RC.
	//         Refer to the Application Note AN3300 for more details on how to
	//         calibrate the MSI.
	// @param  _MSICALIBRATIONVALUE_ specifies the calibration trimming value.
	//         (default is RCC_MSICALIBRATION_DEFAULT).
	//         This parameter must be a number between 0 and 0xFF.
	//
	#define RCC_MSI_CALIBRATIONVALUE_ADJUST(_MSICALIBRATIONVALUE_) \
			(MODIFY_REG(RCC->ICSCR, RCC_ICSCR_MSITRIM, (uint32_t)(_MSICALIBRATIONVALUE_) << 24))

	// @brief  Macro to enable Internal Multi Speed oscillator (MSI).
	// @note   After enabling the MSI, the application software should wait on MSIRDY
	//         flag to be set indicating that MSI clock is stable and can be used as
	//         system clock source.
	//
	#define RCC_MSI_ENABLE()  SET_BIT(RCC->CR, RCC_CR_MSION)

	// @brief  Macro to disable the Internal Multi Speed oscillator (MSI).
	// @note   The MSI is stopped by hardware when entering STOP and STANDBY modes.
	//         It is used (enabled by hardware) as system clock source after startup
	//         from Reset, wakeup from STOP and STANDBY mode, or in case of failure
	//         of the HSE used directly or indirectly as system clock (if the Clock
	//         Security System CSS is enabled).
	// @note   MSI can not be stopped if it is used as system clock source. In this case,
	//         you have to select another source of the system clock then stop the MSI.
	// @note   When the MSI is stopped, MSIRDY flag goes low after 6 MSI oscillator
	//         clock cycles.
	//
	#define RCC_MSI_DISABLE() CLEAR_BIT(RCC->CR, RCC_CR_MSION)

	// @defgroup RCC_APB1_Clock_Enable_Disable_Status APB1 Peripheral Clock Enabled or Disabled Status
	// @brief  Check whether the APB1 peripheral clock is enabled or not.
	// @note   After reset, the peripheral clock (used for registers read/write access)
	//         is disabled and the application software has to enable this clock before
	//         using it.
	//
	#define RCC_PWR_IS_CLK_ENABLED()         (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN) != RESET)
	#define RCC_PWR_IS_CLK_DISABLED()        (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN) == RESET)

	// @brief  Macro to get the Internal Multi Speed oscillator (MSI) clock range in run mode
	// @retval MSI clock range.
	//         This parameter must be one of the following values:
	//     @arg @ref RCC_MSIRANGE_0 MSI clock is around 65.536 KHz
	//     @arg @ref RCC_MSIRANGE_1 MSI clock is around 131.072 KHz
	//     @arg @ref RCC_MSIRANGE_2 MSI clock is around 262.144 KHz
	//     @arg @ref RCC_MSIRANGE_3 MSI clock is around 524.288 KHz
	//     @arg @ref RCC_MSIRANGE_4 MSI clock is around 1.048 MHz
	//     @arg @ref RCC_MSIRANGE_5 MSI clock is around 2.097 MHz (default after Reset or wake-up from STANDBY)
	//     @arg @ref RCC_MSIRANGE_6 MSI clock is around 4.194 MHz
	//
	#define RCC_GET_MSI_RANGE() (uint32_t)(READ_BIT(RCC->ICSCR, RCC_ICSCR_MSIRANGE))

	//
	// @brief  Macro to configure the External High Speed oscillator (HSE).
	// @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
	//         supported by this macro. User should request a transition to HSE Off
	//         first and then HSE On or HSE Bypass.
	// @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
	//         software should wait on HSERDY flag to be set indicating that HSE clock
	//         is stable and can be used to clock the PLL and/or system clock.
	// @note   HSE state can not be changed if it is used directly or through the
	//         PLL as system clock. In this case, you have to select another source
	//         of the system clock then change the HSE state (ex. disable it).
	// @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
	// @note   This function reset the CSSON bit, so if the clock security system(CSS)
	//         was previously enabled you have to enable it again after calling this
	//         function.
	// @param  __STATE__ specifies the new state of the HSE.
	//          This parameter can be one of the following values:
	//            @arg @ref RCC_HSE_OFF turn OFF the HSE oscillator, HSERDY flag goes low after
	//                              6 HSE oscillator clock cycles.
	//            @arg @ref RCC_HSE_ON turn ON the HSE oscillator
	//            @arg @ref RCC_HSE_BYPASS HSE oscillator bypassed with external clock
	//
	#define RCC_HSE_CONFIG(__STATE__)                                     \
					  do{                                                     \
						__IO uint32_t tmpreg;                                 \
						if ((__STATE__) == RCC_HSE_ON)                        \
						{                                                     \
						  SET_BIT(RCC->CR, RCC_CR_HSEON);                     \
						}                                                     \
						else if ((__STATE__) == RCC_HSE_BYPASS)               \
						{                                                     \
						  SET_BIT(RCC->CR, RCC_CR_HSEBYP);                    \
						  SET_BIT(RCC->CR, RCC_CR_HSEON);                     \
						}                                                     \
						else                                                  \
						{                                                     \
						  CLEAR_BIT(RCC->CR, RCC_CR_HSEON);                   \
						  /* Delay after an RCC peripheral clock */           \
						  tmpreg = READ_BIT(RCC->CR, RCC_CR_HSEON);           \
						  CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);                  \
						}                                                     \
						UNUSED(tmpreg);										  \
					  }while(0)

	// @brief  Macro to adjust the Internal High Speed oscillator (HSI) calibration value.
	// @note   The calibration is used to compensate for the variations in voltage
	//         and temperature that influence the frequency of the internal HSI RC.
	// @param  _HSICALIBRATIONVALUE_ specifies the calibration trimming value.
	//         (default is RCC_HSICALIBRATION_DEFAULT).
	//         This parameter must be a number between 0 and 0x1F.
	//
	#define RCC_HSI_CALIBRATIONVALUE_ADJUST(_HSICALIBRATIONVALUE_) \
			(MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSITRIM, (uint32_t)(_HSICALIBRATIONVALUE_) << 8))

	// @brief  Macro to enable or disable the Internal High Speed oscillator (HSI).
	// @note     After enabling the HSI, the application software should wait on
	//           HSIRDY flag to be set indicating that HSI clock is stable and can
	//           be used to clock the PLL and/or system clock.
	// @note     HSI can not be stopped if it is used directly or through the PLL
	//           as system clock. In this case, you have to select another source
	//           of the system clock then stop the HSI.
	// @note     The HSI is stopped by hardware when entering STOP and STANDBY modes.
	// @param    __STATE__ specifies the new state of the HSI.
	//           This parameter can be one of the following values:
	//            @arg @ref RCC_HSI_OFF turn OFF the HSI oscillator
	//            @arg @ref RCC_HSI_ON turn ON the HSI oscillator
	//            @arg @ref RCC_HSI_DIV4 turn ON the HSI oscillator and divide it by 4
	// @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
	//         clock cycles.
	//
	#define RCC_HSI_CONFIG(__STATE__) \
					MODIFY_REG(RCC->CR, RCC_CR_HSION | RCC_CR_HSIDIVEN , (uint32_t)(__STATE__))

	// @brief  Macros to enable or disable the Internal High Speed oscillator (HSI).
	// @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
	//         It is used (enabled by hardware) as system clock source after startup
	//         from Reset, wakeup from STOP and STANDBY mode, or in case of failure
	//         of the HSE used directly or indirectly as system clock (if the Clock
	//         Security System CSS is enabled).
	// @note   HSI can not be stopped if it is used as system clock source. In this case,
	//         you have to select another source of the system clock then stop the HSI.
	// @note   After enabling the HSI, the application software should wait on HSIRDY
	//         flag to be set indicating that HSI clock is stable and can be used as
	//         system clock source.
	// @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
	//         clock cycles.
	//
	#define RCC_HSI_ENABLE()  SET_BIT(RCC->CR, RCC_CR_HSION)
	#define RCC_HSI_DISABLE() CLEAR_BIT(RCC->CR, RCC_CR_HSION)

	#define RCC_PWR_CLK_DISABLE()     CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_PWREN))

	//
	// @brief  Set the FLASH Latency.
	// @param  __LATENCY__ FLASH Latency
	//          This parameter can be one of the following values:
	//            @arg @ref FLASH_LATENCY_0  FLASH Zero Latency cycle
	//            @arg @ref FLASH_LATENCY_1  FLASH One Latency cycle
	// @retval none
	//
	#define FLASH_SET_LATENCY(__LATENCY__) \
					  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)(__LATENCY__))

	// @brief  Get oscillator clock selected as PLL input clock
	// @retval The clock source used for PLL entry. The returned value can be one
	//         of the following:
	//             @arg @ref RCC_PLLSOURCE_HSI HSI oscillator clock selected as PLL input clock
	//             @arg @ref RCC_PLLSOURCE_HSE HSE oscillator clock selected as PLL input clock
	//
	#define RCC_GET_PLL_OSCSOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLSRC)))

	// @brief  Macro to get the clock source used as system clock.
	// @retval The clock source used as system clock. The returned value can be one
	//         of the following:
	//             @arg @ref RCC_SYSCLKSOURCE_STATUS_MSI MSI used as system clock
	//             @arg @ref RCC_SYSCLKSOURCE_STATUS_HSI HSI used as system clock
	//             @arg @ref RCC_SYSCLKSOURCE_STATUS_HSE HSE used as system clock
	//             @arg @ref RCC_SYSCLKSOURCE_STATUS_PLLCLK PLL used as system clock
	//
	#define RCC_GET_SYSCLK_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR,RCC_CFGR_SWS)))

	//
	// Private variables
	//

	//
	// Private function declarations
	//

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
	static StatusTypeDef RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency);

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
	StatusTypeDef RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);

	//
	// @brief  Configures the SysTick clock source.
	// @param  CLKSource: specifies the SysTick clock source.
	//          This parameter can be one of the following values:
	//             @arg SYSTICK_CLKSOURCE_HCLK_DIV8: AHB clock divided by 8 selected as SysTick clock source.
	//             @arg SYSTICK_CLKSOURCE_HCLK: AHB clock selected as SysTick clock source.
	// @retval None
	//
	void SYSTICK_CLKSourceConfig(uint32_t CLKSource);

	//
	// @brief  Update number of Flash wait states in line with MSI range and current
	//		 voltage range
	// @param  MSIrange  MSI range value from RCC_MSIRANGE_0 to RCC_MSIRANGE_6
	// @retval status
	//
	static StatusTypeDef RCC_SetFlashLatencyFromMSIRange(uint32_t MSIrange);

	//
	// Public variable initialization
	//

	//
	// Namespace body
	//

	void RCC_GPIOx_CLK_ENABLE(GPIO_TypeDef* const GPIOx) {
		uint32_t bit = 0U;
		if (GPIOx == GPIOA) {
			bit = RCC_IOPENR_GPIOAEN;
		} else if (GPIOx == GPIOB) {
			bit = RCC_IOPENR_GPIOBEN;
		} else if (GPIOx == GPIOC) {
			bit = RCC_IOPENR_GPIOCEN;
		} else if (GPIOx == GPIOD) {
			bit = RCC_IOPENR_GPIODEN;
		} else if (GPIOx == GPIOH) {
			bit = RCC_IOPENR_GPIOHEN;
		} else {
			bit = 0U;
		}
		if (bit != 0U) {
			SET_BIT(RCC->IOPENR, bit);
			/* Delay after an RCC peripheral clock enabling */
			__IO uint32_t tmpreg = READ_BIT(RCC->IOPENR, bit);
			UNUSED(tmpreg);
		}
	}

	void RCC_GPIOA_CLK_ENABLE() {
		RCC_GPIOx_CLK_ENABLE(GPIOA);
	}
	void RCC_GPIOB_CLK_ENABLE() {
		RCC_GPIOx_CLK_ENABLE(GPIOB);
	}
	void RCC_GPIOC_CLK_ENABLE() {
		RCC_GPIOx_CLK_ENABLE(GPIOC);
	}
	void RCC_GPIOH_CLK_ENABLE() {
		RCC_GPIOx_CLK_ENABLE(GPIOH);
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
					tickstart = Timers::GetTick();

					// Wait till HSE is ready
					while (RCC_GetFlag(RCC_FLAG_HSERDY) == RESET) {
						if ((Timers::GetTick() - tickstart) > HSE_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}
				} else {
					// Get Start Tick
					tickstart = Timers::GetTick();

					// Wait till HSE is disabled
					while (RCC_GetFlag(RCC_FLAG_HSERDY) != RESET) {
						if ((Timers::GetTick() - tickstart) > HSE_TIMEOUT_VALUE) {
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
					tickstart = Timers::GetTick();

					// Wait till HSI is ready
					while (RCC_GetFlag(RCC_FLAG_HSIRDY) == RESET) {
						if ((Timers::GetTick() - tickstart) > HSI_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}

					// Adjusts the Internal High Speed oscillator (HSI) calibration value.
					RCC_HSI_CALIBRATIONVALUE_ADJUST(RCC_OscInitStruct->HSICalibrationValue);
				} else {
					// Disable the Internal High Speed oscillator (HSI).
					RCC_HSI_DISABLE();

					// Get Start Tick
					tickstart = Timers::GetTick();

					// Wait till HSI is disabled
					while (RCC_GetFlag(RCC_FLAG_HSIRDY) != RESET) {
						if ((Timers::GetTick() - tickstart) > HSI_TIMEOUT_VALUE) {
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
					Timers::InitTick(TICK_INT_PRIORITY);
				}
			} else {
				// Check the MSI State
				if (RCC_OscInitStruct->MSIState != RCC_MSI_OFF) {
					// Enable the Multi Speed oscillator (MSI).
					RCC_MSI_ENABLE();

					// Get Start Tick
					tickstart = Timers::GetTick();

					// Wait till MSI is ready
					while (RCC_GetFlag(RCC_FLAG_MSIRDY) == RESET) {
						if ((Timers::GetTick() - tickstart) > MSI_TIMEOUT_VALUE) {
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
					tickstart = Timers::GetTick();

					// Wait till MSI is ready
					while (RCC_GetFlag(RCC_FLAG_MSIRDY) != RESET) {
						if ((Timers::GetTick() - tickstart) > MSI_TIMEOUT_VALUE) {
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
				tickstart = Timers::GetTick();

				// Wait till LSI is ready
				while (RCC_GetFlag(RCC_FLAG_LSIRDY) == RESET) {
					if ((Timers::GetTick() - tickstart) > LSI_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else {
				// Disable the Internal Low Speed oscillator (LSI).
				RCC_LSI_DISABLE();

				// Get Start Tick
				tickstart = Timers::GetTick();

				// Wait till LSI is disabled
				while (RCC_GetFlag(RCC_FLAG_LSIRDY) != RESET) {
					if ((Timers::GetTick() - tickstart) > LSI_TIMEOUT_VALUE) {
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
				tickstart = Timers::GetTick();

				while (IS_BIT_CLR(PWR->CR, PWR_CR_DBP)) {
					if ((Timers::GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			}

			// Set the new LSE configuration -----------------------------------------
			RCC_LSE_CONFIG(RCC_OscInitStruct->LSEState);
			// Check the LSE State
			if (RCC_OscInitStruct->LSEState != RCC_LSE_OFF) {
				// Get Start Tick
				tickstart = Timers::GetTick();

				// Wait till LSE is ready
				while (RCC_GetFlag(RCC_FLAG_LSERDY) == RESET) {
					if ((Timers::GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else {
				// Get Start Tick
				tickstart = Timers::GetTick();

				// Wait till LSE is disabled
				while (RCC_GetFlag(RCC_FLAG_LSERDY) != RESET) {
					if ((Timers::GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			}

			// Require to disable power clock if necessary
			if (pwrclkchanged == SET) {
				RCC_PWR_CLK_DISABLE();
			}
		}

		//----------------------------- HSI48 Configuration --------------------------
		if (((RCC_OscInitStruct->OscillatorType) & RCC_OSCILLATORTYPE_HSI48)
			== RCC_OSCILLATORTYPE_HSI48) {
			// Check the HSI48 State
			if (RCC_OscInitStruct->HSI48State != RCC_HSI48_OFF) {
				// Enable the Internal High Speed oscillator (HSI48).
				RCC_HSI48_ENABLE()
				;

				// Get Start Tick
				tickstart = Timers::GetTick();

				// Wait till HSI48 is ready
				while (RCC_GetFlag(RCC_FLAG_HSI48RDY) == RESET) {
					if ((Timers::GetTick() - tickstart) > HSI48_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else {
				// Disable the Internal High Speed oscillator (HSI48).
				RCC_HSI48_DISABLE()
				;

				// Get Start Tick
				tickstart = Timers::GetTick();

				// Wait till HSI48 is ready
				while (RCC_GetFlag(RCC_FLAG_HSI48RDY) != RESET) {
					if ((Timers::GetTick() - tickstart) > HSI48_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			}
		}

		//-------------------------------- PLL Configuration -----------------------
		if ((RCC_OscInitStruct->PLL.PLLState) != RCC_PLL_NONE) {
			// Check if the PLL is used as system clock or not
			if (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK) {
				if ((RCC_OscInitStruct->PLL.PLLState) == RCC_PLL_ON) {
					// Disable the main PLL.
					RCC_PLL_DISABLE();

					// Get Start Tick
					tickstart = Timers::GetTick();

					// Wait till PLL is disabled
					while (RCC_GetFlag(RCC_FLAG_PLLRDY) != RESET) {
						if ((Timers::GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}

					// Configure the main PLL clock source, multiplication and division factors.
					RCC_PLL_CONFIG(RCC_OscInitStruct->PLL.PLLSource, RCC_OscInitStruct->PLL.PLLMUL,
						RCC_OscInitStruct->PLL.PLLDIV);
					// Enable the main PLL.
					RCC_PLL_ENABLE();

					// Get Start Tick
					tickstart = Timers::GetTick();

					// Wait till PLL is ready
					while (RCC_GetFlag(RCC_FLAG_PLLRDY) == RESET) {
						if ((Timers::GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
							return STATUS_TIMEOUT;
						}
					}
				} else {
					// Disable the main PLL.
					RCC_PLL_DISABLE();

					// Get Start Tick
					tickstart = Timers::GetTick();

					// Wait till PLL is disabled
					while (RCC_GetFlag(RCC_FLAG_PLLRDY) != RESET) {
						if ((Timers::GetTick() - tickstart) > PLL_TIMEOUT_VALUE) {
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
			tickstart = Timers::GetTick();

			if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSE) {
				while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSE) {
					if ((Timers::GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_PLLCLK) {
				while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_PLLCLK) {
					if ((Timers::GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else if (RCC_ClkInitStruct->SYSCLKSource == RCC_SYSCLKSOURCE_HSI) {
				while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_HSI) {
					if ((Timers::GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			} else {
				while (RCC_GET_SYSCLK_SOURCE() != RCC_SYSCLKSOURCE_STATUS_MSI) {
					if ((Timers::GetTick() - tickstart) > CLOCKSWITCH_TIMEOUT_VALUE) {
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
		Timers::InitTick(TICK_INT_PRIORITY);

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
			|| (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LCD) == RCC_PERIPHCLK_LCD)
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
				tickstart = Timers::GetTick();

				while (IS_BIT_CLR(PWR->CR, PWR_CR_DBP)) {
					if ((Timers::GetTick() - tickstart) > RCC_DBP_TIMEOUT_VALUE) {
						return STATUS_TIMEOUT;
					}
				}
			}

			// Check if user wants to change HSE RTC prescaler whereas HSE is enabled
			temp_reg = (RCC->CR & RCC_CR_RTCPRE);
			if ((temp_reg != (PeriphClkInit->RTCClockSelection & RCC_CR_RTCPRE))
				|| (temp_reg != (PeriphClkInit->LCDClockSelection & RCC_CR_RTCPRE))
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
					|| ((temp_reg != (PeriphClkInit->LCDClockSelection & RCC_CSR_RTCSEL))
						&& (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_LCD)
							== RCC_PERIPHCLK_LCD))
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
					tickstart = Timers::GetTick();

					// Wait till LSE is ready
					while (RCC_GetFlag(RCC_FLAG_LSERDY) == RESET) {
						if ((Timers::GetTick() - tickstart) > RCC_LSE_TIMEOUT_VALUE) {
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

		//------------------------------- USART1 Configuration ------------------------
		if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USART1) == RCC_PERIPHCLK_USART1) {
			// Configure the USART1 clock source
			RCC_USART1_CONFIG(PeriphClkInit->Usart1ClockSelection);
		}

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

		//---------------------------- USB and RNG configuration --------------------
		if (((PeriphClkInit->PeriphClockSelection) & RCC_PERIPHCLK_USB) == (RCC_PERIPHCLK_USB)) {
			RCC_USB_CONFIG(PeriphClkInit->UsbClockSelection);
		}

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
		uint32_t pllmul = 0U, plldiv = 0U, pllvco = 0U;

		switch (PeriphClk) {
			case RCC_PERIPHCLK_RTC:
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

	//
	// @brief  Configures the RCC_OscInitStruct according to the internal
	// RCC configuration registers.
	// @param  RCC_OscInitStruct pointer to an RCC_OscInitTypeDef structure that
	// will be configured.
	// @retval None
	//
	void RCC_GetOscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
	{
		// Set all possible values for the Oscillator type parameter ---------------
		RCC_OscInitStruct->OscillatorType = RCC_OSCILLATORTYPE_HSE |
			RCC_OSCILLATORTYPE_HSI |
			RCC_OSCILLATORTYPE_LSE |
			RCC_OSCILLATORTYPE_LSI |
			RCC_OSCILLATORTYPE_MSI;
		#if defined(RCC_HSI48_SUPPORT)
			RCC_OscInitStruct->OscillatorType |= RCC_OSCILLATORTYPE_HSI48;
		#endif // RCC_HSI48_SUPPORT


		// Get the HSE configuration -----------------------------------------------
		if((RCC->CR &RCC_CR_HSEBYP) == RCC_CR_HSEBYP)
		{
			RCC_OscInitStruct->HSEState = RCC_HSE_BYPASS;
		}
		else if((RCC->CR &RCC_CR_HSEON) == RCC_CR_HSEON)
		{
			RCC_OscInitStruct->HSEState = RCC_HSE_ON;
		}
		else
		{
			RCC_OscInitStruct->HSEState = RCC_HSE_OFF;
		}

		// Get the HSI configuration -----------------------------------------------
		if((RCC->CR &RCC_CR_HSION) == RCC_CR_HSION)
		{
			RCC_OscInitStruct->HSIState = RCC_HSI_ON;
		}
		else
		{
			RCC_OscInitStruct->HSIState = RCC_HSI_OFF;
		}

		RCC_OscInitStruct->HSICalibrationValue = (uint32_t)((RCC->ICSCR & RCC_ICSCR_HSITRIM) >> 8);

		// Get the MSI configuration -----------------------------------------------
		if((RCC->CR &RCC_CR_MSION) == RCC_CR_MSION)
		{
			RCC_OscInitStruct->MSIState = RCC_MSI_ON;
		}
		else
		{
			RCC_OscInitStruct->MSIState = RCC_MSI_OFF;
		}

		RCC_OscInitStruct->MSICalibrationValue = (uint32_t)((RCC->ICSCR & RCC_ICSCR_MSITRIM) >> RCC_ICSCR_MSITRIM_BITNUMBER);
		RCC_OscInitStruct->MSIClockRange = (uint32_t)((RCC->ICSCR & RCC_ICSCR_MSIRANGE));

		// Get the LSE configuration -----------------------------------------------
		if((RCC->CSR &RCC_CSR_LSEBYP) == RCC_CSR_LSEBYP)
		{
			RCC_OscInitStruct->LSEState = RCC_LSE_BYPASS;
		}
		else if((RCC->CSR &RCC_CSR_LSEON) == RCC_CSR_LSEON)
		{
			RCC_OscInitStruct->LSEState = RCC_LSE_ON;
		}
		else
		{
			RCC_OscInitStruct->LSEState = RCC_LSE_OFF;
		}

		// Get the LSI configuration -----------------------------------------------
		if((RCC->CSR &RCC_CSR_LSION) == RCC_CSR_LSION)
		{
			RCC_OscInitStruct->LSIState = RCC_LSI_ON;
		}
		else
		{
			RCC_OscInitStruct->LSIState = RCC_LSI_OFF;
		}

	  	#if defined(RCC_HSI48_SUPPORT)
			// Get the HSI48 configuration if any-----------------------------------------
	  	  	RCC_OscInitStruct->HSI48State = RCC_GET_HSI48_STATE();
	  	#endif // RCC_HSI48_SUPPORT

	  	// Get the PLL configuration -----------------------------------------------
	  	if((RCC->CR &RCC_CR_PLLON) == RCC_CR_PLLON)
	  	{
	  		RCC_OscInitStruct->PLL.PLLState = RCC_PLL_ON;
	  	}
	  	else
	  	{
	  		RCC_OscInitStruct->PLL.PLLState = RCC_PLL_OFF;
	  	}
	  	RCC_OscInitStruct->PLL.PLLSource = (uint32_t)(RCC->CFGR & RCC_CFGR_PLLSRC);
	  	RCC_OscInitStruct->PLL.PLLMUL = (uint32_t)(RCC->CFGR & RCC_CFGR_PLLMUL);
	  	RCC_OscInitStruct->PLL.PLLDIV = (uint32_t)(RCC->CFGR & RCC_CFGR_PLLDIV);
	}

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
	uint8_t RCC_GetFlag(uint8_t flag) {
		uint8_t res = 0U;
		uint32_t reg;
		if ((flag >> 5) == CR_REG_INDEX) {
			reg = RCC->CR;
		} else if ((flag >> 5) == CSR_REG_INDEX) {
			reg = RCC->CSR;
		} else {
			reg = RCC->CRRCR;
		}
		if ((reg & (1U << (flag & RCC_FLAG_MASK))) != 0) {
			res = 1U;
		}
		return res;
	}

	//
	// @brief  Update number of Flash wait states in line with MSI range and current
	//		 voltage range
	// @param  MSIrange  MSI range value from RCC_MSIRANGE_0 to RCC_MSIRANGE_6
	// @retval status
	//
	static StatusTypeDef RCC_SetFlashLatencyFromMSIRange(uint32_t MSIrange)
	{
		uint32_t vos = 0;
		uint32_t latency = FLASH_LATENCY_0;  // default value 0WS

		// HCLK can reach 4 MHz only if AHB prescaler = 1
		if (READ_BIT(RCC->CFGR, RCC_CFGR_HPRE) == RCC_SYSCLK_DIV1)
		{
			if(RCC_PWR_IS_CLK_ENABLED())
			{
				vos = READ_BIT(PWR->CR, PWR_CR_VOS);
			}
			else
			{
				RCC_PWR_CLK_ENABLE();
				vos = READ_BIT(PWR->CR, PWR_CR_VOS);
				RCC_PWR_CLK_DISABLE();
			}

			// Check if need to set latency 1 only for Range 3 & HCLK = 4MHz
			if((vos == PWR_REGULATOR_VOLTAGE_SCALE3) && (MSIrange == RCC_MSIRANGE_6))
			{
				latency = FLASH_LATENCY_1; // 1WS
			}
		}

		FLASH_SET_LATENCY(latency);

		// Check that the new number of wait states is taken into account to access the Flash
		// memory by reading the FLASH_ACR register
		if((FLASH->ACR & FLASH_ACR_LATENCY) != latency)
		{
			return STATUS_ERROR;
		}

		return STATUS_OK;
	}

	float32_t CYCLES_PER_MICROSECOND(void) {
		return SystemCoreClock / 1000000L; // 16 or 20
	}

	// End

}
