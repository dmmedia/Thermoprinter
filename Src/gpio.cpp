/*
 * gpio.cpp
 *
 *  Created on: 17. dets 2017
 *      Author: Den
 */

#include <stdint.h>
#include <stm32l0xx.h>
#include "macros.h"
#include "gpio.h"
#include "typedefs.h"
#include "rcc.h"

namespace GPIO {
	//
	// Private definitions and constants
	//

	#define GPIO_MODE             (static_cast<uint32_t>(0x00000003U))
	#define GPIO_OUTPUT_TYPE      (static_cast<uint32_t>(0x00000010U))
	#define GPIO_MODE_IT          (static_cast<uint32_t>(0x00010000U))
	#define GPIO_MODE_EVT         (static_cast<uint32_t>(0x00020000U))
	#define RISING_EDGE           (static_cast<uint32_t>(0x00100000U))
	#define FALLING_EDGE          (static_cast<uint32_t>(0x00200000U))
	#define EXTI_MODE             (static_cast<uint32_t>(0x10000000U))

	#define GPIO_GET_INDEX(__GPIOx__)    (((__GPIOx__) == (GPIOA))? 0U :\
										  (((__GPIOx__) == (GPIOB))? 1U :\
										  (((__GPIOx__) == (GPIOC))? 2U :\
										  (((__GPIOx__) == (GPIOD))? 3U :\
										  (((__GPIOx__) == (GPIOH))? 5U : 6U)))))

	//
	// Private variables
	//

	//
	// Private function declarations
	//

	static void GPIO_EXTI_IRQHandler(const uint16_t GPIO_Pin);

	//
	// Public variable initialization
	//

	//
	// Namespace body
	//

	//
	// @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
	// @param  GPIOx: where x can be (A..E and H) to select the GPIO peripheral for STM32L0XX family devices.
	//                Note that GPIOE is not available on all devices.
	// @param  GPIO_Init: pointer to a GPIO_InitTypeDef structure that contains
	//                    the configuration information for the specified GPIO peripheral.
	// @retval None
	//
	void GpioInit(GPIO_TypeDef* const GPIOx, const GpioInit_t* const GPIO_Init)
	{
		uint32_t position = 0x00U;
		uint32_t iocurrent = 0x00U;
		uint32_t temp = 0x00U;

		// Configure the port pins
		while (((GPIO_Init->Pin) >> position) != 0U)
		{
			// Get the IO position
			iocurrent = (GPIO_Init->Pin) & (1U << position);

			if(iocurrent != 0U)
			{
				//--------------------- GPIO Mode Configuration ------------------------
				// In case of Alternate function mode selection
				if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
				{
					// Configure Alternate function mapped with the current IO
					temp = GPIOx->AFR[position >> 3U];
					temp &= ~(static_cast<uint32_t>(0xFU) << ((position & 0x07U) * 4U)) ;
					temp |= (static_cast<uint32_t>((GPIO_Init->Alternate) << ((position & 0x07U) * 4U))) ;
					GPIOx->AFR[position >> 3U] = temp;
				}

				// In case of Output or Alternate function mode selection
				if((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
						(GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
				{
					// Configure the IO Speed
					temp = GPIOx->OSPEEDR;
					//lint -save -e835 -e1960 -e9033
					temp &= ~(static_cast<uint32_t>(GPIO_OSPEEDER_OSPEED0) << (position * 2U));
					//lint -restore
					temp |= (GPIO_Init->Speed << (position * 2U));
					GPIOx->OSPEEDR = temp;

					// Configure the IO Output Type
					temp= GPIOx->OTYPER;
					temp &= ~(static_cast<uint32_t>(GPIO_OTYPER_OT_0) << position) ;
					temp |= (((GPIO_Init->Mode & GPIO_OUTPUT_TYPE) >> 4U) << position);
					GPIOx->OTYPER = temp;
				}

				// Configure IO Direction mode (Input, Output, Alternate or Analog)
				temp = GPIOx->MODER;
				//lint -save -e835 -e1960 -e9033
				temp &= ~(static_cast<uint32_t>(GPIO_MODER_MODE0) << (position * 2U));
				//lint -restore
				temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
				GPIOx->MODER = temp;

				// Activate the Pull-up or Pull down resistor for the current IO
				temp = GPIOx->PUPDR;
				//lint -save -e835 -e1960 -e9033
				temp &= ~(static_cast<uint32_t>(GPIO_PUPDR_PUPD0) << (position * 2U));
				//lint -restore
				temp |= ((GPIO_Init->Pull) << (position * 2U));
				GPIOx->PUPDR = temp;

				//--------------------- EXTI Mode Configuration ------------------------
				// Configure the External Interrupt or event for the current IO
				if((GPIO_Init->Mode & EXTI_MODE) == EXTI_MODE)
				{
					// Enable SYSCFG Clock
					//lint -save -e1924 -e9078 -e923 -e835
					RCC_SYSCFG_CLK_ENABLE();

					temp = SYSCFG->EXTICR[position >> 2U];
					//lint -restore
					CLEAR_BIT(temp, static_cast<uint32_t>(0x0FU) << (4U * (position & 0x03U)));
					//lint -save -e1924 -e835 -e9078 -e923 -e9033 -e1960
					SET_BIT(temp, static_cast<uint32_t>(GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U)));
					SYSCFG->EXTICR[position >> 2U] = temp;

					// Clear EXTI line configuration
					temp = EXTI->IMR;
					//lint -restore
					temp &= ~(iocurrent);
					if((GPIO_Init->Mode & GPIO_MODE_IT) == GPIO_MODE_IT)
					{
						temp |= iocurrent;
					}
					//lint -save -e1924 -e9078 -e923
					EXTI->IMR = temp;

					temp = EXTI->EMR;
					//lint -restore
					temp &= ~(iocurrent);
					if((GPIO_Init->Mode & GPIO_MODE_EVT) == GPIO_MODE_EVT)
					{
						temp |= iocurrent;
					}
					//lint -save -e1924 -e9078 -e923
					EXTI->EMR = temp;

					// Clear Rising Falling edge configuration
					temp = EXTI->RTSR;
					//lint -restore
					temp &= ~(iocurrent);
					if((GPIO_Init->Mode & RISING_EDGE) == RISING_EDGE)
					{
						temp |= iocurrent;
					}
					//lint -save -e1924 -e9078 -e923
					EXTI->RTSR = temp;

					temp = EXTI->FTSR;
					//lint -restore
					temp &= ~(iocurrent);
					if((GPIO_Init->Mode & FALLING_EDGE) == FALLING_EDGE)
					{
						temp |= iocurrent;
					}
					//lint -save -e1924 -e9078 -e923
					EXTI->FTSR = temp;
					//lint -restore
				}
			}
			position++;
		}
	}

	//
	// @brief  De-initializes the GPIOx peripheral registers to their default reset values.
	// @param  GPIOx: where x can be (A..E and H) to select the GPIO peripheral for STM32L0XX family devices.
	//                Note that GPIOE is not available on all devices.
	// @param  GPIO_Pin: specifies the port bit to be written.
	//                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
	//                   All port bits are not necessarily available on all GPIOs.
	// @retval None
	//
	void GpioDeInit(GPIO_TypeDef  * const GPIOx, const uint32_t GPIO_Pin)
	{
		uint32_t position = 0x00U;
		uint32_t iocurrent = 0x00U;
		uint32_t tmp = 0x00U;

		// Configure the port pins
		while ((GPIO_Pin >> position) != 0U)
		{
			// Get the IO position
			iocurrent = (GPIO_Pin) & (static_cast<uint32_t>(1U) << position);

			if(iocurrent != 0U)
			{
				//------------------------- GPIO Mode Configuration --------------------
				// Configure IO Direction in Input Floting Mode
				//lint -save -e835 -e1960 -e9033
				GPIOx->MODER |= (static_cast<uint32_t>(GPIO_MODER_MODE0) << (position * 2U));
				//lint -restore

				// Configure the default Alternate Function in current IO
				GPIOx->AFR[position >> 3U] &= ~(static_cast<uint32_t>(0xFU) << ((position & 0x07U) * 4U)) ;

				// Configure the default value for IO Speed
				//lint -save -e835 -e1960 -e9033
				GPIOx->OSPEEDR &= ~(static_cast<uint32_t>(GPIO_OSPEEDER_OSPEED0) << (position * 2U));
				//lint -restore

				// Configure the default value IO Output Type
				GPIOx->OTYPER  &= ~(static_cast<uint32_t>(GPIO_OTYPER_OT_0) << position) ;

				// Deactivate the Pull-up oand Pull-down resistor for the current IO
				//lint -save -e1960 -e835 -e9033
				GPIOx->PUPDR &= ~(static_cast<uint32_t>(GPIO_PUPDR_PUPD0) << (position * 2U));
				//lint -restore

				//------------------------- EXTI Mode Configuration --------------------
				// Clear the External Interrupt or Event for the current IO

				//lint -save -e1924 -e9078 -e923
				tmp = SYSCFG->EXTICR[position >> 2U];
				//lint -restore
				tmp &= (static_cast<uint32_t>(0x0FU) << (4U * (position & 0x03U)));
				//lint -save -e1924 -e835 -e9078 -e923 -e9033 -e1960
				if(tmp == (static_cast<uint32_t>(GPIO_GET_INDEX(GPIOx)) << (4U * (position & 0x03U))))
				{
					tmp = static_cast<uint32_t>(0x0FU) << (4U * (position & 0x03U));
					SYSCFG->EXTICR[position >> 2U] &= ~tmp;

					// Clear EXTI line configuration
					EXTI->IMR &= ~(iocurrent);
					EXTI->EMR &= ~(iocurrent);

					// Clear Rising Falling edge configuration
					EXTI->RTSR &= ~(iocurrent);
					EXTI->FTSR &= ~(iocurrent);
				}
				//lint -restore
			}
			position++;
		}
	}

	void setInput(GPIO_TypeDef  * const port, const uint32_t pin, const uint32_t mode) {
		GpioInit_t GPIO_InitStruct;

		GPIO_InitStruct.Pin = pin;
		GPIO_InitStruct.Mode = mode;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GpioInit(port, &GPIO_InitStruct);
	}  // set to input, which allows it to be pulled high by pullups

	void setOutput(GPIO_TypeDef  * const port, const uint32_t pin) {
	  GpioInit_t GPIO_InitStruct;

	  GPIO_InitStruct.Pin = pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	  GpioInit(port, &GPIO_InitStruct);
	}  // set to output

	//
	// @brief  This function handles EXTI interrupt request.
	// @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
	// @retval None
	//
	static void GPIO_EXTI_IRQHandler(const uint16_t GPIO_Pin)
	{
		// EXTI line interrupt detected
		//lint -save -e1924 -e9078 -e923
		if((EXTI->PR & GPIO_Pin) != 0U)
		{
			EXTI->PR = GPIO_Pin;
			GPIO_EXTI_Callback(GPIO_Pin);
		}
		//lint -restore
	}

	bool pinExists(GPIO_TypeDef * const GPIOx, const uint32_t GPIO_Pin) {
		return IsGpioPinAvailable(GPIOx, GPIO_Pin);
	}

	// End
}

extern "C" {
	void EXTI0_1_IRQHandler(void);
	void EXTI2_3_IRQHandler(void);
	void EXTI4_15_IRQHandler(void);

	//
	// @brief This function handles EXTI line 0 to 1 interrupts.
	//
	void EXTI0_1_IRQHandler(void)
	{
		GPIO::GPIO_EXTI_IRQHandler(GPIO::GPIO_PIN_1);
	}

	//
	// @brief This function handles EXTI line 2 to 3 interrupts.
	//
	void EXTI2_3_IRQHandler(void)
	{
		GPIO::GPIO_EXTI_IRQHandler(GPIO::GPIO_PIN_2);
	}

	//
	// @brief This function handles EXTI line 4 to 15 interrupts.
	//
	void EXTI4_15_IRQHandler(void)
	{
		GPIO::GPIO_EXTI_IRQHandler(GPIO::GPIO_PIN_4);
		GPIO::GPIO_EXTI_IRQHandler(GPIO::GPIO_PIN_5);
		GPIO::GPIO_EXTI_IRQHandler(GPIO::GPIO_PIN_6);
		GPIO::GPIO_EXTI_IRQHandler(GPIO::GPIO_PIN_13);
	}

	//
	// @brief  EXTI line detection callbacks.
	// @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
	// @retval None
	//lint -save -e1960 -e129
	__weak void GPIO_EXTI_Callback(const uint16_t GPIO_Pin)
	{
	  // Prevent unused argument(s) compilation warning
	  UNUSED(GPIO_Pin);

	  // NOTE: This function Should not be modified, when the callback is needed,
	  //       the HAL_GPIO_EXTI_Callback could be implemented in the user file
	  //
	}
	//lint -restore
}
