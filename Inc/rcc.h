/*
 * rcc.h
 *
 *  Created on: 17. dets 2017
 *      Author: Den
 */

#ifndef RCC_H_
#define RCC_H_

#include <stdint.h>

/** @defgroup RCC_Oscillator_Type Oscillator Type
  * @{
  */
#define RCC_OSCILLATORTYPE_NONE            ((uint32_t)0x00000000)
#define RCC_OSCILLATORTYPE_HSE             ((uint32_t)0x00000001)
#define RCC_OSCILLATORTYPE_HSI             ((uint32_t)0x00000002)
#define RCC_OSCILLATORTYPE_LSE             ((uint32_t)0x00000004)
#define RCC_OSCILLATORTYPE_LSI             ((uint32_t)0x00000008)
#define RCC_OSCILLATORTYPE_MSI             ((uint32_t)0x00000010)
#if defined(RCC_HSI48_SUPPORT)
#define RCC_OSCILLATORTYPE_HSI48           ((uint32_t)0x00000020)
#endif /* RCC_HSI48_SUPPORT */

#define RCC_HSI_OFF                      ((uint32_t)0x00000000)           /*!< HSI clock deactivation */
#define RCC_HSI_ON                       RCC_CR_HSION                     /*!< HSI clock activation */
#define RCC_HSI_DIV4                     (RCC_CR_HSIDIVEN | RCC_CR_HSION) /*!< HSI_DIV4 clock activation */
#if defined(RCC_CR_HSIOUTEN)
#define RCC_HSI_OUTEN                    RCC_CR_HSIOUTEN                  /*!< HSI_OUTEN clock activation */
#endif /* RCC_CR_HSIOUTEN */

/* Bits position in  in the CFGR register */
#define RCC_CFGR_PLLMUL_BITNUMBER         RCC_CFGR_PLLMUL_Pos
#define RCC_CFGR_PLLDIV_BITNUMBER         RCC_CFGR_PLLDIV_Pos
#define RCC_CFGR_HPRE_BITNUMBER           RCC_CFGR_HPRE_Pos
#define RCC_CFGR_PPRE1_BITNUMBER          RCC_CFGR_PPRE1_Pos
#define RCC_CFGR_PPRE2_BITNUMBER          RCC_CFGR_PPRE2_Pos
/* Bits position in  in the ICSCR register */
#define RCC_ICSCR_MSIRANGE_BITNUMBER      RCC_ICSCR_MSIRANGE_Pos
#define RCC_ICSCR_MSITRIM_BITNUMBER       RCC_ICSCR_MSITRIM_Pos

#define RCC_PLL_NONE                      ((uint32_t)0x00000000)  /*!< PLL is not configured */
#define RCC_PLL_OFF                       ((uint32_t)0x00000001)  /*!< PLL deactivation */
#define RCC_PLL_ON                        ((uint32_t)0x00000002)  /*!< PLL activation */

/** @brief  Macro to get the clock source used as system clock.
  * @retval The clock source used as system clock. The returned value can be one
  *         of the following:
  *             @arg @ref RCC_SYSCLKSOURCE_STATUS_MSI MSI used as system clock
  *             @arg @ref RCC_SYSCLKSOURCE_STATUS_HSI HSI used as system clock
  *             @arg @ref RCC_SYSCLKSOURCE_STATUS_HSE HSE used as system clock
  *             @arg @ref RCC_SYSCLKSOURCE_STATUS_PLLCLK PLL used as system clock
  */
#define RCC_GET_SYSCLK_SOURCE() ((uint32_t)(READ_BIT(RCC->CFGR,RCC_CFGR_SWS)))

#define RCC_SYSCLKSOURCE_STATUS_MSI      RCC_CFGR_SWS_MSI            /*!< MSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSI      RCC_CFGR_SWS_HSI            /*!< HSI used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_HSE      RCC_CFGR_SWS_HSE            /*!< HSE used as system clock */
#define RCC_SYSCLKSOURCE_STATUS_PLLCLK   RCC_CFGR_SWS_PLL            /*!< PLL used as system clock */

/** @brief  Get oscillator clock selected as PLL input clock
  * @retval The clock source used for PLL entry. The returned value can be one
  *         of the following:
  *             @arg @ref RCC_PLLSOURCE_HSI HSI oscillator clock selected as PLL input clock
  *             @arg @ref RCC_PLLSOURCE_HSE HSE oscillator clock selected as PLL input clock
  */
#define RCC_GET_PLL_OSCSOURCE() ((uint32_t)(READ_BIT(RCC->CFGR, RCC_CFGR_PLLSRC)))

#define RCC_PLLSOURCE_HSI           RCC_CFGR_PLLSRC_HSI        /*!< HSI clock selected as PLL entry clock source */
#define RCC_PLLSOURCE_HSE           RCC_CFGR_PLLSRC_HSE        /*!< HSE clock selected as PLL entry clock source */

/* Defines used for Flags */
#define CR_REG_INDEX                     ((uint8_t)1)
#define CSR_REG_INDEX                    ((uint8_t)2)
#define CRRCR_REG_INDEX                  ((uint8_t)3)

#define RCC_FLAG_MASK                    ((uint8_t)0x1F)

/** @brief  Check RCC flag is set or not.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *     @arg @ref RCC_FLAG_HSIRDY HSI oscillator clock ready
  *     @arg @ref RCC_FLAG_HSI48RDY HSI48 oscillator clock ready (not available on all devices)
  *     @arg @ref RCC_FLAG_HSIDIV HSI16 divider flag
  *     @arg @ref RCC_FLAG_MSIRDY MSI oscillator clock ready
  *     @arg @ref RCC_FLAG_HSERDY HSE oscillator clock ready
  *     @arg @ref RCC_FLAG_PLLRDY PLL clock ready
  *     @arg @ref RCC_FLAG_LSECSS LSE oscillator clock CSS detected
  *     @arg @ref RCC_FLAG_LSERDY LSE oscillator clock ready
  *     @arg @ref RCC_FLAG_FWRST Firewall reset
  *     @arg @ref RCC_FLAG_LSIRDY LSI oscillator clock ready
  *     @arg @ref RCC_FLAG_OBLRST Option Byte Loader (OBL) reset
  *     @arg @ref RCC_FLAG_PINRST Pin reset
  *     @arg @ref RCC_FLAG_PORRST POR/PDR reset
  *     @arg @ref RCC_FLAG_SFTRST Software reset
  *     @arg @ref RCC_FLAG_IWDGRST Independent Watchdog reset
  *     @arg @ref RCC_FLAG_WWDGRST Window Watchdog reset
  *     @arg @ref RCC_FLAG_LPWRRST Low Power reset
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#if defined(RCC_HSI48_SUPPORT)
#define RCC_GET_FLAG(__FLAG__) (((((((((__FLAG__) >> 5) == CR_REG_INDEX)? RCC->CR :((((__FLAG__) >> 5) == CSR_REG_INDEX) ? RCC->CSR :RCC->CRRCR)))) & ((uint32_t)1 << ((__FLAG__) & RCC_FLAG_MASK))) != 0 ) ? 1 : 0 )
#else
#define RCC_GET_FLAG(__FLAG__) (((((((((__FLAG__) >> 5) == CR_REG_INDEX)? RCC->CR : RCC->CSR))) & ((uint32_t)1 << ((__FLAG__) & RCC_FLAG_MASK))) != 0 ) ? 1 : 0 )
#endif /* RCC_HSI48_SUPPORT */

/** @defgroup RCC_Flag Flags
  *        Elements values convention: XXXYYYYYb
  *           - YYYYY  : Flag position in the register
  *           - XXX  : Register index
  *                 - 001: CR register
  *                 - 010: CSR register
  *                 - 011: CRRCR register (*)
  * (*)   Applicable only for STM32L052xx, STM32L053xx, (...), STM32L073xx & STM32L082xx
  * @{
  */
/* Flags in the CR register */
#define RCC_FLAG_HSIRDY                  ((uint8_t)((CR_REG_INDEX << 5) | 2))     /*!< Internal High Speed clock ready flag */
#define RCC_FLAG_HSIDIV                  ((uint8_t)((CR_REG_INDEX << 5) | 4))     /*!< HSI16 divider flag */
#define RCC_FLAG_MSIRDY                  ((uint8_t)((CR_REG_INDEX << 5) | 9))     /*!< MSI clock ready flag */
#define RCC_FLAG_HSERDY                  ((uint8_t)((CR_REG_INDEX << 5) | 17))    /*!< External High Speed clock ready flag */
#define RCC_FLAG_PLLRDY                  ((uint8_t)((CR_REG_INDEX << 5) | 25))    /*!< PLL clock ready flag */
/* Flags in the CSR register */
#define RCC_FLAG_LSIRDY                  ((uint8_t)((CSR_REG_INDEX << 5) | 1))    /*!< Internal Low Speed oscillator Ready */
#define RCC_FLAG_LSERDY                  ((uint8_t)((CSR_REG_INDEX << 5) | 9)) /*!< External Low Speed oscillator Ready */
#define RCC_FLAG_LSECSS                  ((uint8_t)((CSR_REG_INDEX << 5) | 14))   /*!< CSS on LSE failure Detection */
#define RCC_FLAG_OBLRST                  ((uint8_t)((CSR_REG_INDEX << 5) | 25))   /*!< Options bytes loading reset flag */
#define RCC_FLAG_PINRST                  ((uint8_t)((CSR_REG_INDEX << 5) | 26))   /*!< PIN reset flag */
#define RCC_FLAG_PORRST                  ((uint8_t)((CSR_REG_INDEX << 5) | 27))   /*!< POR/PDR reset flag */
#define RCC_FLAG_SFTRST                  ((uint8_t)((CSR_REG_INDEX << 5) | 28))   /*!< Software Reset flag */
#define RCC_FLAG_IWDGRST                 ((uint8_t)((CSR_REG_INDEX << 5) | 29))   /*!< Independent Watchdog reset flag */
#define RCC_FLAG_WWDGRST                 ((uint8_t)((CSR_REG_INDEX << 5) | 30))   /*!< Window watchdog reset flag */
#define RCC_FLAG_LPWRRST                 ((uint8_t)((CSR_REG_INDEX << 5) | 31))   /*!< Low-Power reset flag */
#if defined(RCC_CSR_FWRSTF)
#define RCC_FLAG_FWRST                   ((uint8_t)((CSR_REG_INDEX << 5) |  8))   /*!< RCC flag FW reset */
#endif /* RCC_CSR_FWRSTF */
/* Flags in the CRRCR register */
#if defined(RCC_HSI48_SUPPORT)
#define RCC_FLAG_HSI48RDY                ((uint8_t)((CRRCR_REG_INDEX << 5) | 1))  /*!< HSI48 clock ready flag */
#endif /* RCC_HSI48_SUPPORT */

/* ########################## Oscillator Values adaptation ####################*/
/**
  * @brief Adjust the value of External High Speed oscillator (HSE) used in your application.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSE is used as system clock source, directly or through the PLL).
  */
#if !defined  (HSE_VALUE)
  #define HSE_VALUE    ((uint32_t)8000000U) /*!< Value of the External oscillator in Hz */
#endif /* HSE_VALUE */

#if !defined  (HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT    ((uint32_t)100U)   /*!< Time out for HSE start up, in ms */
#endif /* HSE_STARTUP_TIMEOUT */

/**
  * @brief Internal Multiple Speed oscillator (MSI) default value.
  *        This value is the default MSI range value after Reset.
  */
#if !defined  (MSI_VALUE)
  #define MSI_VALUE    ((uint32_t)2097000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* MSI_VALUE */

/**
  * @brief Internal High Speed oscillator (HSI) value.
  *        This value is used by the RCC HAL module to compute the system frequency
  *        (when HSI is used as system clock source, directly or through the PLL).
  */
#if !defined  (HSI_VALUE)
  #define HSI_VALUE    ((uint32_t)16000000U) /*!< Value of the Internal oscillator in Hz*/
#endif /* HSI_VALUE */

/**
  * @brief Internal High Speed oscillator for USB (HSI48) value.
  */
#if !defined  (HSI48_VALUE)
#define HSI48_VALUE ((uint32_t)48000000U) /*!< Value of the Internal High Speed oscillator for USB in Hz.
                                             The real value may vary depending on the variations
                                             in voltage and temperature.  */
#endif /* HSI48_VALUE */

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */
#if !defined  (LSI_VALUE)
 #define LSI_VALUE  ((uint32_t)37000U)       /*!< LSI Typical Value in Hz*/
#endif /* LSI_VALUE */                      /*!< Value of the Internal Low Speed oscillator in Hz
                                             The real value may vary depending on the variations
                                             in voltage and temperature.*/
/**
  * @brief External Low Speed oscillator (LSE) value.
  *        This value is used by the UART, RTC HAL module to compute the system frequency
  */
#if !defined  (LSE_VALUE)
  #define LSE_VALUE    ((uint32_t)32768U) /*!< Value of the External oscillator in Hz*/
#endif /* LSE_VALUE */

#if !defined  (LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT  ((uint32_t)5000U)   /*!< Time out for LSE start up, in ms */
#endif /* LSE_STARTUP_TIMEOUT */

/** @defgroup RCC_Timeout RCC Timeout
  * @{
  */
/* Disable Backup domain write protection state change timeout */
#define RCC_DBP_TIMEOUT_VALUE      (100U)       /* 100 ms */
/* LSE state change timeout */
#define RCC_LSE_TIMEOUT_VALUE      LSE_STARTUP_TIMEOUT
#define CLOCKSWITCH_TIMEOUT_VALUE  (5000U)  /* 5 s    */
#define HSE_TIMEOUT_VALUE          HSE_STARTUP_TIMEOUT
#define MSI_TIMEOUT_VALUE          (2U)      /* 2 ms (minimum Tick + 1) */
#define HSI_TIMEOUT_VALUE          (2U)      /* 2 ms (minimum Tick + 1) */
#define HSI48_TIMEOUT_VALUE        (2U)      /* 2 ms (minimum Tick + 1) */
#define LSI_TIMEOUT_VALUE          (2U)      /* 2 ms (minimum Tick + 1) */
#define PLL_TIMEOUT_VALUE          (2U)      /* 2 ms (minimum Tick + 1) */
#if defined(RCC_HSI48_SUPPORT)
#define HSI48_TIMEOUT_VALUE        (2U)      /* 2 ms (minimum Tick + 1) */
#endif /* RCC_HSI48_SUPPORT */

/** @defgroup FLASH_Latency FLASH Latency
  * @{
  */
#define FLASH_LATENCY_0            ((uint32_t)0x00000000U)    /*!< FLASH Zero Latency cycle */
#define FLASH_LATENCY_1            FLASH_ACR_LATENCY         /*!< FLASH One Latency cycle */

/** @defgroup RCC_AHB_Clock_Source AHB Clock Source
  * @{
  */
#define RCC_SYSCLK_DIV1                  RCC_CFGR_HPRE_DIV1   /*!< SYSCLK not divided */
#define RCC_SYSCLK_DIV2                  RCC_CFGR_HPRE_DIV2   /*!< SYSCLK divided by 2 */
#define RCC_SYSCLK_DIV4                  RCC_CFGR_HPRE_DIV4   /*!< SYSCLK divided by 4 */
#define RCC_SYSCLK_DIV8                  RCC_CFGR_HPRE_DIV8   /*!< SYSCLK divided by 8 */
#define RCC_SYSCLK_DIV16                 RCC_CFGR_HPRE_DIV16  /*!< SYSCLK divided by 16 */
#define RCC_SYSCLK_DIV64                 RCC_CFGR_HPRE_DIV64  /*!< SYSCLK divided by 64 */
#define RCC_SYSCLK_DIV128                RCC_CFGR_HPRE_DIV128 /*!< SYSCLK divided by 128 */
#define RCC_SYSCLK_DIV256                RCC_CFGR_HPRE_DIV256 /*!< SYSCLK divided by 256 */
#define RCC_SYSCLK_DIV512                RCC_CFGR_HPRE_DIV512 /*!< SYSCLK divided by 512 */

#define PWR_REGULATOR_VOLTAGE_SCALE1   PWR_CR_VOS_0
#define PWR_REGULATOR_VOLTAGE_SCALE2   PWR_CR_VOS_1
#define PWR_REGULATOR_VOLTAGE_SCALE3   PWR_CR_VOS

/**
  * @brief  Set the FLASH Latency.
  * @param  __LATENCY__ FLASH Latency
  *          This parameter can be one of the following values:
  *            @arg @ref FLASH_LATENCY_0  FLASH Zero Latency cycle
  *            @arg @ref FLASH_LATENCY_1  FLASH One Latency cycle
  * @retval none
  */
#define FLASH_SET_LATENCY(__LATENCY__) \
                  MODIFY_REG(FLASH->ACR, FLASH_ACR_LATENCY, (uint32_t)(__LATENCY__))

#define  TICK_INT_PRIORITY            ((uint32_t)0U)    /*!< tick interrupt priority */

/**
* @brief This function configures the source of the time base.
*        The time source is configured  to have 1ms time base with a dedicated
*        Tick interrupt priority.
* @note This function is called  automatically at the beginning of program after
*       reset by Init() or at any time when clock is reconfigured  by RCC_ClockConfig().
* @note In the default implementation, SysTick timer is the source of time base.
*       It is used to generate interrupts at regular time intervals.
*       Care must be taken if HAL_Delay() is called from a peripheral ISR process,
*       The the SysTick interrupt must have higher priority (numerically lower)
*       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
*       The function is declared as __Weak  to be overwritten  in case of other
*       implementation  in user file.
* @param TickPriority: Tick interrupt priority.
* @retval status
*/
StatusTypeDef InitTick(uint32_t TickPriority);

/** @defgroup RCC_APB1_Clock_Enable_Disable APB1 Peripheral Clock Enable Disable
* @brief  Enable or disable the Low Speed APB (APB1) peripheral clock.
* @note   After reset, the peripheral clock (used for registers read/write access)
*         is disabled and the application software has to enable this clock before
*         using it.
* @{
*/
#define RCC_WWDG_CLK_ENABLE()    SET_BIT(RCC->APB1ENR, (RCC_APB1ENR_WWDGEN))
#define RCC_PWR_CLK_ENABLE()     SET_BIT(RCC->APB1ENR, (RCC_APB1ENR_PWREN))

#define RCC_WWDG_CLK_DISABLE()    CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_WWDGEN))
#define RCC_PWR_CLK_DISABLE()     CLEAR_BIT(RCC->APB1ENR, (RCC_APB1ENR_PWREN))

/**
* @brief  RCC System, AHB and APB busses clock configuration structure definition
*/
typedef struct
{
uint32_t ClockType;             /*!< The clock to be configured.
									 This parameter can be a value of @ref RCC_System_Clock_Type */

uint32_t SYSCLKSource;          /*!< The clock source (SYSCLKS) used as system clock.
									 This parameter can be a value of @ref RCC_System_Clock_Source */

uint32_t AHBCLKDivider;         /*!< The AHB clock (HCLK) divider. This clock is derived from the system clock (SYSCLK).
									 This parameter can be a value of @ref RCC_AHB_Clock_Source */

uint32_t APB1CLKDivider;        /*!< The APB1 clock (PCLK1) divider. This clock is derived from the AHB clock (HCLK).
									 This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */

uint32_t APB2CLKDivider;        /*!< The APB2 clock (PCLK2) divider. This clock is derived from the AHB clock (HCLK).
									 This parameter can be a value of @ref RCC_APB1_APB2_Clock_Source */
} RCC_ClkInitTypeDef;

/**
* @brief  Returns the SYSCLK frequency
* @note   The system frequency computed by this function is not the real
*         frequency in the chip. It is calculated based on the predefined
*         constant and the selected clock source:
* @note     If SYSCLK source is MSI, function returns a value based on MSI
*             Value as defined by the MSI range.
* @note     If SYSCLK source is HSI, function returns values based on HSI_VALUE(*)
* @note     If SYSCLK source is HSE, function returns a value based on HSE_VALUE(**)
* @note     If SYSCLK source is PLL, function returns a value based on HSE_VALUE(**)
*           or HSI_VALUE(*) multiplied/divided by the PLL factors.
* @note     (*) HSI_VALUE is a constant defined in stm32l0xx_hal_conf.h file (default value
*               16 MHz) but the real value may vary depending on the variations
*               in voltage and temperature.
* @note     (**) HSE_VALUE is a constant defined in stm32l0xx_hal_conf.h file (default value
*                8 MHz), user has to ensure that HSE_VALUE is same as the real
*                frequency of the crystal used. Otherwise, this function may
*                have wrong result.
*
* @note   The result of this function could be not correct when using fractional
*         value for HSE crystal.
*
* @note   This function can be used by the user application to compute the
*         baud-rate for the communication peripherals or configure other parameters.
*
* @note   Each time SYSCLK changes, this function must be called to update the
*         right SYSCLK value. Otherwise, any configuration based on this function will be incorrect.
*
* @retval SYSCLK frequency
*/
uint32_t RCC_GetSysClockFreq(void);

/**
* @brief  RCC PLL configuration structure definition
*/
typedef struct
{
uint32_t PLLState;      /*!< PLLState: The new state of the PLL.
							This parameter can be a value of @ref RCC_PLL_Config */

uint32_t PLLSource;     /*!< PLLSource: PLL entry clock source.
							This parameter must be a value of @ref RCC_PLL_Clock_Source */

uint32_t PLLMUL;        /*!< PLLMUL: Multiplication factor for PLL VCO input clock
							This parameter must be a value of @ref RCC_PLL_Multiplication_Factor*/

uint32_t PLLDIV;        /*!< PLLDIV: Division factor for PLL VCO input clock
							This parameter must be a value of @ref RCC_PLL_Division_Factor*/
} RCC_PLLInitTypeDef;

/**
* @brief  RCC Internal/External Oscillator (HSE, HSI, LSE and LSI) configuration structure definition
*/
typedef struct
{
uint32_t OscillatorType;        /*!< The oscillators to be configured.
									 This parameter can be a value of @ref RCC_Oscillator_Type */

uint32_t HSEState;              /*!< The new state of the HSE.
									 This parameter can be a value of @ref RCC_HSE_Config */

uint32_t LSEState;              /*!< The new state of the LSE.
									 This parameter can be a value of @ref RCC_LSE_Config */

uint32_t HSIState;              /*!< The new state of the HSI.
									 This parameter can be a value of @ref RCC_HSI_Config */

uint32_t HSICalibrationValue;   /*!< The HSI calibration trimming value (default is RCC_HSICALIBRATION_DEFAULT).
									 This parameter must be a number between Min_Data = 0x00 and Max_Data = 0x1F */

uint32_t LSIState;              /*!< The new state of the LSI.
									 This parameter can be a value of @ref RCC_LSI_Config */

#if defined(RCC_HSI48_SUPPORT)
uint32_t HSI48State;            /*!< The new state of the HSI48.
									 This parameter can be a value of @ref RCC_HSI48_Config */

#endif /* RCC_HSI48_SUPPORT */
uint32_t MSIState;              /*!< The new state of the MSI.
									 This parameter can be a value of @ref RCC_MSI_Config */

uint32_t MSICalibrationValue;   /*!< The MSI calibration trimming value. (default is RCC_MSICALIBRATION_DEFAULT).
									 This parameter must be a number between Min_Data = 0x00 and Max_Data = 0xFF */

uint32_t MSIClockRange;         /*!< The MSI  frequency  range.
									  This parameter can be a value of @ref RCC_MSI_Clock_Range */

RCC_PLLInitTypeDef PLL;         /*!< PLL structure parameters */

} RCC_OscInitTypeDef;

/** @defgroup RCC_HSE_Config HSE Config
* @{
*/
#define RCC_HSE_OFF                      ((uint32_t)0x00000000)                     /*!< HSE clock deactivation */
#define RCC_HSE_ON                       RCC_CR_HSEON                               /*!< HSE clock activation */
#define RCC_HSE_BYPASS                   ((uint32_t)(RCC_CR_HSEBYP | RCC_CR_HSEON)) /*!< External clock source for HSE clock */

/**
* @brief  Macro to configure the External High Speed oscillator (HSE).
* @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
*         supported by this macro. User should request a transition to HSE Off
*         first and then HSE On or HSE Bypass.
* @note   After enabling the HSE (RCC_HSE_ON or RCC_HSE_Bypass), the application
*         software should wait on HSERDY flag to be set indicating that HSE clock
*         is stable and can be used to clock the PLL and/or system clock.
* @note   HSE state can not be changed if it is used directly or through the
*         PLL as system clock. In this case, you have to select another source
*         of the system clock then change the HSE state (ex. disable it).
* @note   The HSE is stopped by hardware when entering STOP and STANDBY modes.
* @note   This function reset the CSSON bit, so if the clock security system(CSS)
*         was previously enabled you have to enable it again after calling this
*         function.
* @param  __STATE__ specifies the new state of the HSE.
*          This parameter can be one of the following values:
*            @arg @ref RCC_HSE_OFF turn OFF the HSE oscillator, HSERDY flag goes low after
*                              6 HSE oscillator clock cycles.
*            @arg @ref RCC_HSE_ON turn ON the HSE oscillator
*            @arg @ref RCC_HSE_BYPASS HSE oscillator bypassed with external clock
*/
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

/** @brief  Macro to adjust the Internal High Speed oscillator (HSI) calibration value.
* @note   The calibration is used to compensate for the variations in voltage
*         and temperature that influence the frequency of the internal HSI RC.
* @param  _HSICALIBRATIONVALUE_ specifies the calibration trimming value.
*         (default is RCC_HSICALIBRATION_DEFAULT).
*         This parameter must be a number between 0 and 0x1F.
*/
#define RCC_HSI_CALIBRATIONVALUE_ADJUST(_HSICALIBRATIONVALUE_) \
		(MODIFY_REG(RCC->ICSCR, RCC_ICSCR_HSITRIM, (uint32_t)(_HSICALIBRATIONVALUE_) << 8))

/** @brief  Macro to enable or disable the Internal High Speed oscillator (HSI).
* @note     After enabling the HSI, the application software should wait on
*           HSIRDY flag to be set indicating that HSI clock is stable and can
*           be used to clock the PLL and/or system clock.
* @note     HSI can not be stopped if it is used directly or through the PLL
*           as system clock. In this case, you have to select another source
*           of the system clock then stop the HSI.
* @note     The HSI is stopped by hardware when entering STOP and STANDBY modes.
* @param    __STATE__ specifies the new state of the HSI.
*           This parameter can be one of the following values:
*            @arg @ref RCC_HSI_OFF turn OFF the HSI oscillator
*            @arg @ref RCC_HSI_ON turn ON the HSI oscillator
*            @arg @ref RCC_HSI_DIV4 turn ON the HSI oscillator and divide it by 4
* @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
*         clock cycles.
*/
#define RCC_HSI_CONFIG(__STATE__) \
				MODIFY_REG(RCC->CR, RCC_CR_HSION | RCC_CR_HSIDIVEN , (uint32_t)(__STATE__))

/** @brief  Macros to enable or disable the Internal High Speed oscillator (HSI).
* @note   The HSI is stopped by hardware when entering STOP and STANDBY modes.
*         It is used (enabled by hardware) as system clock source after startup
*         from Reset, wakeup from STOP and STANDBY mode, or in case of failure
*         of the HSE used directly or indirectly as system clock (if the Clock
*         Security System CSS is enabled).
* @note   HSI can not be stopped if it is used as system clock source. In this case,
*         you have to select another source of the system clock then stop the HSI.
* @note   After enabling the HSI, the application software should wait on HSIRDY
*         flag to be set indicating that HSI clock is stable and can be used as
*         system clock source.
* @note   When the HSI is stopped, HSIRDY flag goes low after 6 HSI oscillator
*         clock cycles.
*/
#define RCC_HSI_ENABLE()  SET_BIT(RCC->CR, RCC_CR_HSION)
#define RCC_HSI_DISABLE() CLEAR_BIT(RCC->CR, RCC_CR_HSION)

/** @defgroup RCC_MSI_Config MSI Config
* @{
*/
#define RCC_MSI_OFF                      ((uint32_t)0x00000000)
#define RCC_MSI_ON                       ((uint32_t)0x00000001)

/** @brief  Macro to get the Internal Multi Speed oscillator (MSI) clock range in run mode
* @retval MSI clock range.
*         This parameter must be one of the following values:
*     @arg @ref RCC_MSIRANGE_0 MSI clock is around 65.536 KHz
*     @arg @ref RCC_MSIRANGE_1 MSI clock is around 131.072 KHz
*     @arg @ref RCC_MSIRANGE_2 MSI clock is around 262.144 KHz
*     @arg @ref RCC_MSIRANGE_3 MSI clock is around 524.288 KHz
*     @arg @ref RCC_MSIRANGE_4 MSI clock is around 1.048 MHz
*     @arg @ref RCC_MSIRANGE_5 MSI clock is around 2.097 MHz (default after Reset or wake-up from STANDBY)
*     @arg @ref RCC_MSIRANGE_6 MSI clock is around 4.194 MHz
*/
#define RCC_GET_MSI_RANGE() (uint32_t)(READ_BIT(RCC->ICSCR, RCC_ICSCR_MSIRANGE))

/** @defgroup RCC_MSI_Clock_Range MSI Clock Range
* @{
*/
#define RCC_MSIRANGE_0                   RCC_ICSCR_MSIRANGE_0 /*!< MSI = 65.536 KHz  */
#define RCC_MSIRANGE_1                   RCC_ICSCR_MSIRANGE_1 /*!< MSI = 131.072 KHz */
#define RCC_MSIRANGE_2                   RCC_ICSCR_MSIRANGE_2 /*!< MSI = 262.144 KHz */
#define RCC_MSIRANGE_3                   RCC_ICSCR_MSIRANGE_3 /*!< MSI = 524.288 KHz */
#define RCC_MSIRANGE_4                   RCC_ICSCR_MSIRANGE_4 /*!< MSI = 1.048 MHz   */
#define RCC_MSIRANGE_5                   RCC_ICSCR_MSIRANGE_5 /*!< MSI = 2.097 MHz   */
#define RCC_MSIRANGE_6                   RCC_ICSCR_MSIRANGE_6 /*!< MSI = 4.194 MHz   */

/** @defgroup RCC_APB1_Clock_Enable_Disable_Status APB1 Peripheral Clock Enabled or Disabled Status
* @brief  Check whether the APB1 peripheral clock is enabled or not.
* @note   After reset, the peripheral clock (used for registers read/write access)
*         is disabled and the application software has to enable this clock before
*         using it.
* @{
*/
#define RCC_WWDG_IS_CLK_ENABLED()        (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN) != RESET)
#define RCC_PWR_IS_CLK_ENABLED()         (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN) != RESET)
#define RCC_WWDG_IS_CLK_DISABLED()       (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_WWDGEN) == RESET)
#define RCC_PWR_IS_CLK_DISABLED()        (READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN) == RESET)

/**
* @brief  Update number of Flash wait states in line with MSI range and current
		 voltage range
* @param  MSIrange  MSI range value from RCC_MSIRANGE_0 to RCC_MSIRANGE_6
* @retval status
*/
static StatusTypeDef RCC_SetFlashLatencyFromMSIRange(uint32_t MSIrange)
{
uint32_t vos = 0;
uint32_t latency = FLASH_LATENCY_0;  /* default value 0WS */

/* HCLK can reach 4 MHz only if AHB prescaler = 1 */
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

 /* Check if need to set latency 1 only for Range 3 & HCLK = 4MHz */
 if((vos == PWR_REGULATOR_VOLTAGE_SCALE3) && (MSIrange == RCC_MSIRANGE_6))
 {
   latency = FLASH_LATENCY_1; /* 1WS */
 }
}

FLASH_SET_LATENCY(latency);

/* Check that the new number of wait states is taken into account to access the Flash
  memory by reading the FLASH_ACR register */
if((FLASH->ACR & FLASH_ACR_LATENCY) != latency)
{
 return STATUS_ERROR;
}

return STATUS_OK;
}

/* @brief  Macro to configures the Internal Multi Speed oscillator (MSI) clock range.
* @note     After restart from Reset or wakeup from STANDBY, the MSI clock is
*           around 2.097 MHz. The MSI clock does not change after wake-up from
*           STOP mode.
* @note    The MSI clock range can be modified on the fly.
* @param  _MSIRANGEVALUE_ specifies the MSI Clock range.
*   This parameter must be one of the following values:
*     @arg @ref RCC_MSIRANGE_0 MSI clock is around 65.536 KHz
*     @arg @ref RCC_MSIRANGE_1 MSI clock is around 131.072 KHz
*     @arg @ref RCC_MSIRANGE_2 MSI clock is around 262.144 KHz
*     @arg @ref RCC_MSIRANGE_3 MSI clock is around 524.288 KHz
*     @arg @ref RCC_MSIRANGE_4 MSI clock is around 1.048 MHz
*     @arg @ref RCC_MSIRANGE_5 MSI clock is around 2.097 MHz (default after Reset or wake-up from STANDBY)
*     @arg @ref RCC_MSIRANGE_6 MSI clock is around 4.194 MHz
*/
#define RCC_MSI_RANGE_CONFIG(_MSIRANGEVALUE_) (MODIFY_REG(RCC->ICSCR, \
		RCC_ICSCR_MSIRANGE, (uint32_t)(_MSIRANGEVALUE_)))

/** @brief  Macro adjusts Internal Multi Speed oscillator (MSI) calibration value.
* @note   The calibration is used to compensate for the variations in voltage
*         and temperature that influence the frequency of the internal MSI RC.
*         Refer to the Application Note AN3300 for more details on how to
*         calibrate the MSI.
* @param  _MSICALIBRATIONVALUE_ specifies the calibration trimming value.
*         (default is RCC_MSICALIBRATION_DEFAULT).
*         This parameter must be a number between 0 and 0xFF.
*/
#define RCC_MSI_CALIBRATIONVALUE_ADJUST(_MSICALIBRATIONVALUE_) \
		(MODIFY_REG(RCC->ICSCR, RCC_ICSCR_MSITRIM, (uint32_t)(_MSICALIBRATIONVALUE_) << 24))

/** @brief  Macro to enable Internal Multi Speed oscillator (MSI).
* @note   After enabling the MSI, the application software should wait on MSIRDY
*         flag to be set indicating that MSI clock is stable and can be used as
*         system clock source.
*/
#define RCC_MSI_ENABLE()  SET_BIT(RCC->CR, RCC_CR_MSION)

/** @brief  Macro to disable the Internal Multi Speed oscillator (MSI).
* @note   The MSI is stopped by hardware when entering STOP and STANDBY modes.
*         It is used (enabled by hardware) as system clock source after startup
*         from Reset, wakeup from STOP and STANDBY mode, or in case of failure
*         of the HSE used directly or indirectly as system clock (if the Clock
*         Security System CSS is enabled).
* @note   MSI can not be stopped if it is used as system clock source. In this case,
*         you have to select another source of the system clock then stop the MSI.
* @note   When the MSI is stopped, MSIRDY flag goes low after 6 MSI oscillator
*         clock cycles.
*/
#define RCC_MSI_DISABLE() CLEAR_BIT(RCC->CR, RCC_CR_MSION)

/** @defgroup RCC_LSI_Config LSI Config
* @{
*/
#define RCC_LSI_OFF                      ((uint32_t)0x00000000)   /*!< LSI clock deactivation */
#define RCC_LSI_ON                       RCC_CSR_LSION            /*!< LSI clock activation */

/** @brief Macro to enable the Internal Low Speed oscillator (LSI).
* @note   After enabling the LSI, the application software should wait on
*         LSIRDY flag to be set indicating that LSI clock is stable and can
*         be used to clock the IWDG and/or the RTC.
*/
#define RCC_LSI_ENABLE() SET_BIT(RCC->CSR, RCC_CSR_LSION)

/** @brief Macro to disable the Internal Low Speed oscillator (LSI).
* @note   LSI can not be disabled if the IWDG is running.
* @note   When the LSI is stopped, LSIRDY flag goes low after 6 LSI oscillator
*         clock cycles.
*/
#define RCC_LSI_DISABLE() CLEAR_BIT(RCC->CSR, RCC_CSR_LSION)

/** @defgroup RCC_LSE_Config LSE Config
* @{
*/
#define RCC_LSE_OFF                      ((uint32_t)0x00000000)                       /*!< LSE clock deactivation */
#define RCC_LSE_ON                       RCC_CSR_LSEON                                /*!< LSE clock activation */
#define RCC_LSE_BYPASS                   ((uint32_t)(RCC_CSR_LSEBYP | RCC_CSR_LSEON)) /*!< External clock source for LSE clock */

/**
* @brief  Macro to configure the External Low Speed oscillator (LSE).
* @note Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not supported by this macro.
* @note   As the LSE is in the Backup domain and write access is denied to
*         this domain after reset, you have to enable write access using
*         @ref HAL_PWR_EnableBkUpAccess() function before to configure the LSE
*         (to be done once after reset).
* @note   After enabling the LSE (RCC_LSE_ON or RCC_LSE_BYPASS), the application
*         software should wait on LSERDY flag to be set indicating that LSE clock
*         is stable and can be used to clock the RTC.
* @param  __STATE__ specifies the new state of the LSE.
*         This parameter can be one of the following values:
*            @arg @ref RCC_LSE_OFF turn OFF the LSE oscillator, LSERDY flag goes low after
*                              6 LSE oscillator clock cycles.
*            @arg @ref RCC_LSE_ON turn ON the LSE oscillator.
*            @arg @ref RCC_LSE_BYPASS LSE oscillator bypassed with external clock.
*/
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

#if defined(RCC_HSI48_SUPPORT)
/** @defgroup RCC_HSI48_Config HSI48 Config
  * @{
  */
#define RCC_HSI48_OFF               ((uint8_t)0x00)
#define RCC_HSI48_ON                ((uint8_t)0x01)
#endif /* RCC_HSI48_SUPPORT */

  /**
	* @brief  Enable the Internal High Speed oscillator for USB (HSI48).
	* @note   After enabling the HSI48, the application software should wait on
	*         HSI48RDY flag to be set indicating that HSI48 clock is stable and can
	*         be used to clock the USB.
	* @note   The HSI48 is stopped by hardware when entering STOP and STANDBY modes.
	*/
  #define RCC_HSI48_ENABLE()  do { SET_BIT(RCC->CRRCR, RCC_CRRCR_HSI48ON);            \
										 SET_BIT(RCC->APB2ENR, RCC_APB2ENR_SYSCFGEN);       \
										 SET_BIT(SYSCFG->CFGR3, SYSCFG_CFGR3_ENREF_HSI48);  \
									} while (0)
  /**
	* @brief  Disable the Internal High Speed oscillator for USB (HSI48).
	*/
  #define RCC_HSI48_DISABLE()  do { CLEAR_BIT(RCC->CRRCR, RCC_CRRCR_HSI48ON);   \
										  CLEAR_BIT(SYSCFG->CFGR3, SYSCFG_CFGR3_ENREF_HSI48);  \
									 } while (0)

  /** @brief Macro to disable the main PLL.
	* @note   The main PLL can not be disabled if it is used as system clock source
	*/
  #define RCC_PLL_DISABLE() CLEAR_BIT(RCC->CR, RCC_CR_PLLON)

  /** @brief Macro to configure the main PLL clock source, multiplication and division factors.
	* @note   This function must be used only when the main PLL is disabled.
	*
	* @param  __RCC_PLLSOURCE__ specifies the PLL entry clock source.
	*          This parameter can be one of the following values:
	*            @arg @ref RCC_PLLSOURCE_HSI HSI oscillator clock selected as PLL clock entry
	*            @arg @ref RCC_PLLSOURCE_HSE HSE oscillator clock selected as PLL clock entry
	* @param  __PLLMUL__ specifies the multiplication factor for PLL VCO output clock
	*          This parameter can be one of the following values:
	*             @arg @ref RCC_PLL_MUL3   PLLVCO = PLL clock entry x 3
	*             @arg @ref RCC_PLL_MUL4   PLLVCO = PLL clock entry x 4
	*             @arg @ref RCC_PLL_MUL6   PLLVCO = PLL clock entry x 6
	*             @arg @ref RCC_PLL_MUL8   PLLVCO = PLL clock entry x 8
	*             @arg @ref RCC_PLL_MUL12  PLLVCO = PLL clock entry x 12
	*             @arg @ref RCC_PLL_MUL16  PLLVCO = PLL clock entry x 16
	*             @arg @ref RCC_PLL_MUL24  PLLVCO = PLL clock entry x 24
	*             @arg @ref RCC_PLL_MUL32  PLLVCO = PLL clock entry x 32
	*             @arg @ref RCC_PLL_MUL48  PLLVCO = PLL clock entry x 48
	* @note The PLL VCO clock frequency must not exceed 96 MHz when the product is in
	*          Range 1, 48 MHz when the product is in Range 2 and 24 MHz when the product is
	*          in Range 3.
	*
	* @param  __PLLDIV__ specifies the division factor for PLL VCO input clock
	*          This parameter can be one of the following values:
	*             @arg @ref RCC_PLL_DIV2 PLL clock output = PLLVCO / 2
	*             @arg @ref RCC_PLL_DIV3 PLL clock output = PLLVCO / 3
	*             @arg @ref RCC_PLL_DIV4 PLL clock output = PLLVCO / 4
	*
	*/
  #define RCC_PLL_CONFIG(__RCC_PLLSOURCE__, __PLLMUL__, __PLLDIV__)\
			MODIFY_REG(RCC->CFGR, (RCC_CFGR_PLLSRC|RCC_CFGR_PLLMUL|RCC_CFGR_PLLDIV),((__RCC_PLLSOURCE__) | (__PLLMUL__) | (__PLLDIV__)))

  /** @brief Macro to enable the main PLL.
	* @note   After enabling the main PLL, the application software should wait on
	*         PLLRDY flag to be set indicating that PLL clock is stable and can
	*         be used as system clock source.
	* @note   The main PLL is disabled by hardware when entering STOP and STANDBY modes.
	*/
  #define RCC_PLL_ENABLE() SET_BIT(RCC->CR, RCC_CR_PLLON)

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

/**
 * @brief  Initializes the RCC Oscillators according to the specified parameters in the
 *         RCC_OscInitTypeDef.
 * @param  RCC_OscInitStruct pointer to an RCC_OscInitTypeDef structure that
 *         contains the configuration information for the RCC Oscillators.
 * @note   The PLL is not disabled when used as system clock.
 * @note   Transitions LSE Bypass to LSE On and LSE On to LSE Bypass are not
 *         supported by this macro. User should request a transition to LSE Off
 *         first and then LSE On or LSE Bypass.
 * @note   Transition HSE Bypass to HSE On and HSE On to HSE Bypass are not
 *         supported by this macro. User should request a transition to HSE Off
 *         first and then HSE On or HSE Bypass.
 * @retval status
 */
StatusTypeDef RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct);

  /**
    * @brief  RCC extended clocks structure definition
    */
  typedef struct
  {
    uint32_t PeriphClockSelection;                /*!< The Extended Clock to be configured.
                                        This parameter can be a value of @ref RCCEx_Periph_Clock_Selection */

    uint32_t RTCClockSelection;         /*!< specifies the RTC clock source.
                                         This parameter can be a value of @ref RCC_RTC_LCD_Clock_Source */

  #if defined(LCD)

    uint32_t LCDClockSelection;         /*!< specifies the LCD clock source.
                                         This parameter can be a value of @ref RCC_RTC_LCD_Clock_Source */

  #endif /* LCD */
  #if defined(RCC_CCIPR_USART1SEL)
    uint32_t Usart1ClockSelection;   /*!< USART1 clock source
                                          This parameter can be a value of @ref RCCEx_USART1_Clock_Source */
  #endif /* RCC_CCIPR_USART1SEL */
    uint32_t Usart2ClockSelection;   /*!< USART2 clock source
                                          This parameter can be a value of @ref RCCEx_USART2_Clock_Source */

    uint32_t Lpuart1ClockSelection;  /*!< LPUART1 clock source
                                          This parameter can be a value of @ref RCCEx_LPUART1_Clock_Source */

    uint32_t I2c1ClockSelection;     /*!< I2C1 clock source
                                          This parameter can be a value of @ref RCCEx_I2C1_Clock_Source */

    uint32_t LptimClockSelection;    /*!< LPTIM1 clock source
                                          This parameter can be a value of @ref RCCEx_LPTIM1_Clock_Source */
  #if defined(USB)
    uint32_t UsbClockSelection;      /*!< Specifies USB and RNG Clock  Selection
                                          This parameter can be a value of @ref RCCEx_USB_Clock_Source */
  #endif /* USB */
  } RCC_PeriphCLKInitTypeDef;

#endif /* RCC_H_ */
