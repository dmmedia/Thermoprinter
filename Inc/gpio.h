#pragma once

/*
 * gpio.h
 *
 *  Created on: 16. dets 2017
 *      Author: Den
 */

#include <stm32l0xx.h>
#include <limits.h>

namespace GPIO {
	/** @defgroup GPIO_mode_define Mode definition
	  * @brief GPIO Configuration Mode
	  *        Elements values convention: 0xX0yz00YZ
	  *           - X  : GPIO mode or EXTI Mode
	  *           - y  : External IT or Event trigger detection
	  *           - z  : IO configuration on External IT or Event
	  *           - Y  : Output type (Push Pull or Open Drain)
	  *           - Z  : IO Direction mode (Input, Output, Alternate or Analog)
	  * @{
	  */
	constexpr uint32_t GPIO_MODE_INPUT 				= 0x00000000U;   /*!< Input Floating Mode                   */
	constexpr uint32_t GPIO_MODE_OUTPUT_PP 			= 0x00000001U;   /*!< Output Push Pull Mode                 */
	constexpr uint32_t GPIO_MODE_OUTPUT_OD 			= 0x00000011U;   /*!< Output Open Drain Mode                */
	constexpr uint32_t GPIO_MODE_AF_PP 				= 0x00000002U;   /*!< Alternate Function Push Pull Mode     */
	constexpr uint32_t GPIO_MODE_AF_OD 				= 0x00000012U;   /*!< Alternate Function Open Drain Mode    */

	constexpr uint32_t GPIO_MODE_ANALOG 			= 0x00000003U;   /*!< Analog Mode  */

	constexpr uint32_t GPIO_MODE_IT_RISING 			= 0x10110000U;   /*!< External Interrupt Mode with Rising edge trigger detection          */
	constexpr uint32_t GPIO_MODE_IT_FALLING 		= 0x10210000U;   /*!< External Interrupt Mode with Falling edge trigger detection         */
	constexpr uint32_t GPIO_MODE_IT_RISING_FALLING 	= 0x10310000U;   /*!< External Interrupt Mode with Rising/Falling edge trigger detection  */

	constexpr uint32_t GPIO_MODE_EVT_RISING 		= 0x10120000U;   /*!< External Event Mode with Rising edge trigger detection               */
	constexpr uint32_t GPIO_MODE_EVT_FALLING		= 0x10220000U;   /*!< External Event Mode with Falling edge trigger detection              */
	constexpr uint32_t GPIO_MODE_EVT_RISING_FALLING	= 0x10320000U;   /*!< External Event Mode with Rising/Falling edge trigger detection       */


	#define GPIO_MODE             (static_cast<uint32_t>(0x00000003U))
	#define GPIO_OUTPUT_TYPE      (static_cast<uint32_t>(0x00000010U))
	#define GPIO_MODE_IT          (static_cast<uint32_t>(0x00010000U))
	#define GPIO_MODE_EVT         (static_cast<uint32_t>(0x00020000U))
	#define RISING_EDGE           (static_cast<uint32_t>(0x00100000U))
	#define FALLING_EDGE          (static_cast<uint32_t>(0x00200000U))
	#define EXTI_MODE             (static_cast<uint32_t>(0x10000000U))

	#define RCC_SYSCFG_CLK_ENABLE()   SET_BIT(RCC->APB2ENR, (RCC_APB2ENR_SYSCFGEN))

	#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0U :\
										  (((__GPIOx__) == (GPIOB))? 1U :\
										  (((__GPIOx__) == (GPIOC))? 2U :\
										  (((__GPIOx__) == (GPIOD))? 3U :\
										  (((__GPIOx__) == (GPIOH))? 5U : 6U)))))

	/** @defgroup GPIO_pins_define Pin definition
	  * @{
	  */
	constexpr uint16_t GPIO_PIN_0 	= 0x0001U;
	constexpr uint16_t GPIO_PIN_1 	= 0x0002U;  /* Pin 1 selected    */
	constexpr uint16_t GPIO_PIN_2 	= 0x0004U;  /* Pin 2 selected    */
	constexpr uint16_t GPIO_PIN_3 	= 0x0008U;  /* Pin 3 selected    */
	constexpr uint16_t GPIO_PIN_4 	= 0x0010U;  /* Pin 4 selected    */
	constexpr uint16_t GPIO_PIN_5 	= 0x0020U;  /* Pin 5 selected    */
	constexpr uint16_t GPIO_PIN_6	= 0x0040U;  /* Pin 6 selected    */
	constexpr uint16_t GPIO_PIN_7	= 0x0080U;  /* Pin 7 selected    */
	constexpr uint16_t GPIO_PIN_8	= 0x0100U;  /* Pin 8 selected    */
	constexpr uint16_t GPIO_PIN_9   = 0x0200U;  /* Pin 9 selected    */
	constexpr uint16_t GPIO_PIN_10  = 0x0400U;  /* Pin 10 selected   */
	constexpr uint16_t GPIO_PIN_11  = 0x0800U;  /* Pin 11 selected   */
	constexpr uint16_t GPIO_PIN_12  = 0x1000U;  /* Pin 12 selected   */
	constexpr uint16_t GPIO_PIN_13  = 0x2000U;  /* Pin 13 selected   */
	constexpr uint16_t GPIO_PIN_14  = 0x4000U;  /* Pin 14 selected   */
	constexpr uint16_t GPIO_PIN_15  = 0x8000U;  /* Pin 15 selected   */
	constexpr uint16_t GPIO_PIN_All = 0xFFFFU;  /* All pins selected */

	 /** @defgroup GPIO_pull_define Pull definition
	   * @brief GPIO Pull-Up or Pull-Down Activation
	   * @{
	   */
	#define  GPIO_NOPULL        (static_cast<uint32_t>(0x00000000U))   /*!< No Pull-up or Pull-down activation  */
	#define  GPIO_PULLUP        (static_cast<uint32_t>(0x00000001U))   /*!< Pull-up activation                  */
	#define  GPIO_PULLDOWN      (static_cast<uint32_t>(0x00000002U))   /*!< Pull-down activation                */

	/** @defgroup GPIO_speed_define Speed definition
	  * @brief GPIO Output Maximum frequency
	  * @{
	  */
	#define  GPIO_SPEED_FREQ_LOW              (static_cast<uint32_t>(0x00000000U))  /*!< range up to 0.4 MHz, please refer to the product datasheet */
	#define  GPIO_SPEED_FREQ_MEDIUM           (static_cast<uint32_t>(0x00000001U))  /*!< range 0.4 MHz to 2 MHz, please refer to the product datasheet */
	#define  GPIO_SPEED_FREQ_HIGH             (static_cast<uint32_t>(0x00000002U))  /*!< range   2 MHz to 10 MHz, please refer to the product datasheet */
	#define  GPIO_SPEED_FREQ_VERY_HIGH        (static_cast<uint32_t>(0x00000003U))  /*!< range  10 MHz to 35 MHz, please refer to the product datasheet */

	/*
	 * Alternate function AF0
	 */
	#define GPIO_AF0_SPI1          (static_cast<uint8_t>(0x00U))  /* SPI1 Alternate Function mapping     */
	#define GPIO_AF0_SPI2          (static_cast<uint8_t>(0x00U))  /* SPI2 Alternate Function mapping     */
	#define GPIO_AF0_USART1        (static_cast<uint8_t>(0x00U))  /* USART1 Alternate Function mapping   */
	#define GPIO_AF0_USART2        (static_cast<uint8_t>(0x00U))  /* USART2 Alternate Function mapping   */
	#define GPIO_AF0_LPUART1       (static_cast<uint8_t>(0x00U))  /* LPUART1 Alternate Function mapping  */
	#define GPIO_AF0_USB           (static_cast<uint8_t>(0x00U))  /* USB Alternate Function mapping      */
	#define GPIO_AF0_LPTIM1        (static_cast<uint8_t>(0x00U))  /* LPTIM1 Alternate Function mapping   */
	#define GPIO_AF0_TSC           (static_cast<uint8_t>(0x00U))  /* TSC Alternate Function mapping      */
	#define GPIO_AF0_TIM2          (static_cast<uint8_t>(0x00U))  /* TIM2 Alternate Function mapping     */
	#define GPIO_AF0_TIM21         (static_cast<uint8_t>(0x00U))  /* TIM21 Alternate Function mapping    */
	#define GPIO_AF0_TIM22         (static_cast<uint8_t>(0x00U))  /* TIM22 Alternate Function mapping    */
	#define GPIO_AF0_EVENTOUT      (static_cast<uint8_t>(0x00U))  /* EVENTOUT Alternate Function mapping */
	#define GPIO_AF0_MCO           (static_cast<uint8_t>(0x00U))  /* MCO Alternate Function mapping      */
	#define GPIO_AF0_SWDIO         (static_cast<uint8_t>(0x00U))  /* SWDIO Alternate Function mapping    */
	#define GPIO_AF0_SWCLK         (static_cast<uint8_t>(0x00U))  /* SWCLK Alternate Function mapping    */
	/**
	  *
	  */

	 /*
	  * Alternate function AF1
	 */
	#define GPIO_AF1_SPI1          (static_cast<uint8_t>(0x01U))  /* SPI1 Alternate Function mapping  */
	#define GPIO_AF1_SPI2          (static_cast<uint8_t>(0x01U))  /* SPI2 Alternate Function mapping  */
	#define GPIO_AF1_I2C1          (static_cast<uint8_t>(0x01U))  /* I2C1 Alternate Function mapping  */
	#define GPIO_AF1_LCD           (static_cast<uint8_t>(0x01U))  /* LCD Alternate Function mapping   */
	/**
	  *
	  */

	/*
	 * Alternate function AF2
	 */
	#define GPIO_AF2_SPI2          (static_cast<uint8_t>(0x02U))  /* SPI2 Alternate Function mapping       */
	#define GPIO_AF2_LPUART1       (static_cast<uint8_t>(0x02U))  /* LPUART1 Alternate Function mapping    */
	#define GPIO_AF2_USB           (static_cast<uint8_t>(0x02U))  /* USB Alternate Function mapping        */
	#define GPIO_AF2_LPTIM1        (static_cast<uint8_t>(0x02U))  /* LPTIM1 Alternate Function mapping     */
	#define GPIO_AF2_TIM2          (static_cast<uint8_t>(0x02U))  /* TIM2 Alternate Function mapping       */
	#define GPIO_AF2_EVENTOUT      (static_cast<uint8_t>(0x02U))  /* EVENTOUT Alternate Function mapping   */
	#define GPIO_AF2_RTC           (static_cast<uint8_t>(0x02U))  /* RTC Alternate Function mapping        */
	/**
	  *
	  */

	/*
	 * Alternate function AF3
	 */
	#define GPIO_AF3_I2C1          (static_cast<uint8_t>(0x03U))  /* I2C1 Alternate Function mapping     */
	#define GPIO_AF3_TSC           (static_cast<uint8_t>(0x03U))  /* TSC  Alternate Function mapping     */
	#define GPIO_AF3_EVENTOUT      (static_cast<uint8_t>(0x03U))  /* EVENTOUT Alternate Function mapping */
	/**
	  *
	  */

	/*
	 * Alternate function AF4
	 */
	#define GPIO_AF4_I2C1            (static_cast<uint8_t>(0x04U))  /* I2C1 Alternate Function mapping     */
	#define GPIO_AF4_USART1          (static_cast<uint8_t>(0x04U))  /* USART1 Alternate Function mapping   */
	#define GPIO_AF4_USART2          (static_cast<uint8_t>(0x04U))  /* USART2 Alternate Function mapping   */
	#define GPIO_AF4_LPUART1         (static_cast<uint8_t>(0x04U))  /* LPUART1 Alternate Function mapping  */
	#define GPIO_AF4_TIM22           (static_cast<uint8_t>(0x04U))  /* TIM22 Alternate Function mapping    */
	#define GPIO_AF4_EVENTOUT        (static_cast<uint8_t>(0x04U))  /* EVENTOUT Alternate Function mapping */
	/**
	  *
	  */

	 /*
	 * Alternate function AF5
	 */
	#define GPIO_AF5_SPI2          (static_cast<uint8_t>(0x05U))  /* SPI2 Alternate Function mapping     */
	#define GPIO_AF5_I2C2          (static_cast<uint8_t>(0x05U))  /* I2C2 Alternate Function mapping     */
	#define GPIO_AF5_TIM2          (static_cast<uint8_t>(0x05U))  /* TIM2 Alternate Function mapping     */
	#define GPIO_AF5_TIM21         (static_cast<uint8_t>(0x05U))  /* TIM21 Alternate Function mapping    */
	#define GPIO_AF5_TIM22         (static_cast<uint8_t>(0x05U))  /* TIM22 Alternate Function mapping    */
	/**
	  *
	  */

	/*
	 * Alternate function AF6
	 */
	#define GPIO_AF6_I2C2          (static_cast<uint8_t>(0x06U))  /* I2C2 Alternate Function mapping      */
	#define GPIO_AF6_TIM21         (static_cast<uint8_t>(0x06U))  /* TIM21 Alternate Function mapping     */
	#define GPIO_AF6_EVENTOUT      (static_cast<uint8_t>(0x06U))  /* EVENTOUT Alternate Function mapping  */
	/**
	  *
	  */

	/*
	 * Alternate function AF7
	 */
	#define GPIO_AF7_COMP1        (static_cast<uint8_t>(0x07U))  /* COMP1 Alternate Function mapping     */
	#define GPIO_AF7_COMP2        (static_cast<uint8_t>(0x07U))  /* COMP2 Alternate Function mapping     */
	/**
	  *
	  */

	/**
	  * @brief  GPIO Bit SET and Bit RESET enumeration
	  */
	typedef enum
	{
	  GPIO_PIN_RESET = 0U,
	  GPIO_PIN_SET
	} GPIO_PinState;

	/**
	  * @brief   GPIO Init structure definition
	  */
	typedef struct
	{
	  uint32_t Pin;       /*!< Specifies the GPIO pins to be configured.
							   This parameter can be a combination of @ref GPIO_pins_define */

	  uint32_t Mode;      /*!< Specifies the operating mode for the selected pins.
							   This parameter can be a value of @ref GPIO_mode_define */

	  uint32_t Pull;      /*!< Specifies the Pull-up or Pull-Down activation for the selected pins.
							   This parameter can be a value of @ref GPIO_pull_define */

	  uint32_t Speed;     /*!< Specifies the speed for the selected pins.
							   This parameter can be a value of @ref GPIO_speed_define */

	  uint32_t Alternate;  /*!< Peripheral to be connected to the selected pins
								This parameter can be a value of @ref GPIOEx_Alternate_function_selection */
	} GpioInit_t;

	/**
	  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
	  * @param  GPIOx: where x can be (A..E and H) to select the GPIO peripheral for STM32L0XX family devices.
	  *                Note that GPIOE is not available on all devices.
	  * @param  GPIO_Init: pointer to a GPIO_InitTypeDef structure that contains
	  *                    the configuration information for the specified GPIO peripheral.
	  * @retval None
	  */
	void GpioInit(GPIO_TypeDef * const GPIOx, const GpioInit_t * const GPIO_Init);

	/**
	  * @brief  De-initializes the GPIOx peripheral registers to their default reset values.
	  * @param  GPIOx: where x can be (A..E and H) to select the GPIO peripheral for STM32L0XX family devices.
	  *                Note that GPIOE is not available on all devices.
	  * @param  GPIO_Pin: specifies the port bit to be written.
	  *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
	  *                   All port bits are not necessarily available on all GPIOs.
	  * @retval None
	  */
	void GpioDeInit(GPIO_TypeDef  * const GPIOx, const uint32_t GPIO_Pin);

	void setOutput(GPIO_TypeDef  * const port, const uint32_t pin);

	/** @defgroup GPIOEx_Pin_Available Pin available
	 * @{
	 */
	#define GPIOA_PIN_AVAILABLE  GPIO_PIN_All
	#define GPIOB_PIN_AVAILABLE  GPIO_PIN_All
	#define GPIOC_PIN_AVAILABLE  GPIO_PIN_All
	#define GPIOD_PIN_AVAILABLE  GPIO_PIN_2
	#define GPIOH_PIN_AVAILABLE  GPIO_PIN_0 | GPIO_PIN_1

	constexpr bool IsGpioPinAvailable(GPIO_TypeDef * const GPIOx, const uint32_t GPIO_Pin) {
		bool res = false;
		if (GPIOx == GPIOA) {
			res = (((GPIO_Pin & GPIOA_PIN_AVAILABLE) != 0U) && ((GPIO_Pin | GPIOA_PIN_AVAILABLE) == GPIOA_PIN_AVAILABLE));
		} else if (GPIOx == GPIOB) {
			res = (((GPIO_Pin & GPIOB_PIN_AVAILABLE) != 0U) && ((GPIO_Pin | GPIOB_PIN_AVAILABLE) == GPIOB_PIN_AVAILABLE));
		} else if (GPIOx == GPIOC) {
			res = (((GPIO_Pin & GPIOC_PIN_AVAILABLE) != 0U) && ((GPIO_Pin | GPIOC_PIN_AVAILABLE) == GPIOC_PIN_AVAILABLE));
		} else if (GPIOx == GPIOD) {
			res = (((GPIO_Pin & GPIOD_PIN_AVAILABLE) != 0U) && ((GPIO_Pin | GPIOD_PIN_AVAILABLE) == GPIOD_PIN_AVAILABLE));
		} else if (GPIOx == GPIOH) {
			res = (((GPIO_Pin & GPIOH_PIN_AVAILABLE) != 0U) && ((GPIO_Pin | GPIOH_PIN_AVAILABLE) == GPIOH_PIN_AVAILABLE));
		} else {
			res = false;
		}
		return res;
	}

	#define PIN_EXISTS(IO) IsGpioPinAvailable(IO ## _PORT, IO ## _PIN)
	constexpr bool pinExists(GPIO_TypeDef * const GPIOx, const uint32_t GPIO_Pin) {
		return IsGpioPinAvailable(GPIOx, GPIO_Pin);
	}

	#define SET_INPUT(IO) GPIO::setInput(IO ## _PORT, IO ## _PIN, GPIO_MODE_INPUT)
	#define SET_INPUT_EXTI(IO) GPIO::setInput(IO ## _PORT, IO ## _PIN, GPIO_MODE_IT_RISING_FALLING)

	#define SET_OUTPUT(IO) GPIO::setOutput(IO ## _PORT, IO ## _PIN)

	/**
	  * @brief  Sets or clears the selected data port bit.
	  *
	  * @note   This function uses GPIOx_BSRR register to allow atomic read/modify
	  *         accesses. In this way, there is no risk of an IRQ occurring between
	  *         the read and the modify access.
	  *
	  * @param  GPIOx: where x can be (A..E and H) to select the GPIO peripheral for STM32L0xx family devices.
	  *                Note that GPIOE is not available on all devices.
	  * @param  GPIO_Pin: specifies the port bit to be written.
	  *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
	  *                   All port bits are not necessarily available on all GPIOs.
	  * @param  PinState: specifies the value to be written to the selected bit.
	  *                   This parameter can be one of the GPIO_PinState enum values:
	  *                        GPIO_PIN_RESET: to clear the port pin
	  *                        GPIO_PIN_SET: to set the port pin
	  * @retval None
	  */
	#define GPIO_WRITE_PIN(GPIOx, GPIO_Pin, PinState) ((PinState != GPIO::GPIO_PIN_RESET) ? (GPIOx->BSRR = GPIO_Pin) : (GPIOx->BRR = GPIO_Pin))
	FORCE_INLINE void writePin(GPIO_TypeDef* const port, uint32_t const pin, GPIO_PinState const state) {
		switch (state) {
			case GPIO_PIN_RESET:
				port->BRR = pin;
				break;
			default:
				port->BSRR = pin;
				break;
		}
	}

	/** @defgroup RCC_IOPORT_Clock_Enable_Disable IOPORT Peripheral Clock Enable Disable
	  * @brief  Enable or disable the IOPORT peripheral clock.
	  * @note   After reset, the peripheral clock (used for registers read/write access)
	  *         is disabled and the application software has to enable this clock before
	  *         using it.
	  * @{
	  */
	#define WRITE(IO,V) GPIO_WRITE_PIN(IO ## _PORT, IO ## _PIN, V)
	#define OUT_WRITE(IO, v) do{ SET_OUTPUT(IO); WRITE(IO, v); }while(0)
	FORCE_INLINE void outWrite(GPIO_TypeDef* const port, uint32_t const pin, GPIO_PinState const state) {
		setOutput(port, pin);
		writePin(port, pin, state);
	}

	/**
	  * @brief  Reads the specified input port pin.
	  * @param  GPIOx: where x can be (A..E and H) to select the GPIO peripheral for STM32L0xx family devices.
	  *                Note that GPIOE is not available on all devices.
	  * @param  GPIO_Pin: specifies the port bit to read.
	  *                   This parameter can be GPIO_PIN_x where x can be (0..15).
	  *                   All port bits are not necessarily available on all GPIOs.
	  * @retval The input port pin value.
	  */
	#define GPIO_READ_PIN(GPIOx, GPIO_PIN) (GPIOx->IDR & GPIO_PIN ? GPIO_PIN_SET : GPIO_PIN_RESET)

	FORCE_INLINE GPIO_PinState read(GPIO_TypeDef* port, uint32_t pin) {
		return GPIO_READ_PIN(port, pin);
	}

	constexpr void RCC_GPIOx_CLK_ENABLE(GPIO_TypeDef* const GPIOx) {
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
	constexpr void RCC_GPIOA_CLK_ENABLE() {RCC_GPIOx_CLK_ENABLE(GPIOA);}
	constexpr void RCC_GPIOB_CLK_ENABLE() {RCC_GPIOx_CLK_ENABLE(GPIOB);}
	constexpr void RCC_GPIOC_CLK_ENABLE() {RCC_GPIOx_CLK_ENABLE(GPIOC);}
	constexpr void RCC_GPIOH_CLK_ENABLE() {RCC_GPIOx_CLK_ENABLE(GPIOH);}

	#define RCC_GPIOA_CLK_DISABLE()        CLEAR_BIT(RCC->IOPENR, RCC_IOPENR_GPIOAEN)
	#define RCC_GPIOB_CLK_DISABLE()        CLEAR_BIT(RCC->IOPENR, RCC_IOPENR_GPIOBEN)
	#define RCC_GPIOC_CLK_DISABLE()        CLEAR_BIT(RCC->IOPENR, RCC_IOPENR_GPIOCEN)
	#define RCC_GPIOH_CLK_DISABLE()        CLEAR_BIT(RCC->IOPENR, RCC_IOPENR_GPIOHEN)

	void setInput(GPIO_TypeDef  *const port, const uint32_t pin, const uint32_t mode);

	void GPIO_EXTI_IRQHandler(const uint16_t GPIO_Pin);
	void GPIO_EXTI_Callback(const uint16_t GPIO_Pin);

	constexpr uint32_t NO_PIN = UINT_MAX;
	GPIO_TypeDef * const NO_PORT = nullptr;

}
