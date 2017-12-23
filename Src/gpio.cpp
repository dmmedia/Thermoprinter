/*
 * gpio.cpp
 *
 *  Created on: 17. dets 2017
 *      Author: Den
 */

#include "gpio.h"
#include "macros.h"

/**
  * @brief  Initializes the GPIOx peripheral according to the specified parameters in the GPIO_Init.
  * @param  GPIOx: where x can be (A..E and H) to select the GPIO peripheral for STM32L0XX family devices.
  *                Note that GPIOE is not available on all devices.
  * @param  GPIO_Init: pointer to a GPIO_InitTypeDef structure that contains
  *                    the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void GpioInit(GPIO_TypeDef *GPIOx, GpioInit_t *GPIO_Init)
{
  uint32_t position = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t temp = 0x00U;

  /* Configure the port pins */
  while (((GPIO_Init->Pin) >> position) != 0)
  {
    /* Get the IO position */
    iocurrent = (GPIO_Init->Pin) & (1U << position);

    if(iocurrent)
    {
      /*--------------------- GPIO Mode Configuration ------------------------*/
      /* In case of Alternate function mode selection */
      if((GPIO_Init->Mode == GPIO_MODE_AF_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
      {
        /* Configure Alternate function mapped with the current IO */
        temp = GPIOx->AFR[position >> 3U];
        temp &= ~((uint32_t)0xFU << ((uint32_t)(position & (uint32_t)0x07U) * 4U)) ;
        temp |= ((uint32_t)(GPIO_Init->Alternate) << (((uint32_t)position & (uint32_t)0x07U) * 4U)) ;
        GPIOx->AFR[position >> 3U] = temp;
      }

      /* In case of Output or Alternate function mode selection */
      if((GPIO_Init->Mode == GPIO_MODE_OUTPUT_PP) || (GPIO_Init->Mode == GPIO_MODE_AF_PP) ||
         (GPIO_Init->Mode == GPIO_MODE_OUTPUT_OD) || (GPIO_Init->Mode == GPIO_MODE_AF_OD))
      {
        /* Configure the IO Speed */
        temp = GPIOx->OSPEEDR;
        temp &= ~(GPIO_OSPEEDER_OSPEED0 << (position * 2U));
        temp |= (GPIO_Init->Speed << (position * 2U));
        GPIOx->OSPEEDR = temp;

        /* Configure the IO Output Type */
        temp= GPIOx->OTYPER;
        temp &= ~(GPIO_OTYPER_OT_0 << position) ;
        temp |= (((GPIO_Init->Mode & GPIO_OUTPUT_TYPE) >> 4U) << position);
        GPIOx->OTYPER = temp;
      }

      /* Configure IO Direction mode (Input, Output, Alternate or Analog) */
      temp = GPIOx->MODER;
      temp &= ~(GPIO_MODER_MODE0 << (position * 2U));
      temp |= ((GPIO_Init->Mode & GPIO_MODE) << (position * 2U));
      GPIOx->MODER = temp;

      /* Activate the Pull-up or Pull down resistor for the current IO */
      temp = GPIOx->PUPDR;
      temp &= ~(GPIO_PUPDR_PUPD0 << (position * 2U));
      temp |= ((GPIO_Init->Pull) << (position * 2U));
      GPIOx->PUPDR = temp;

      /*--------------------- EXTI Mode Configuration ------------------------*/
      /* Configure the External Interrupt or event for the current IO */
      if((GPIO_Init->Mode & EXTI_MODE) == EXTI_MODE)
      {
        /* Enable SYSCFG Clock */
    	  RCC_SYSCFG_CLK_ENABLE();

        temp = SYSCFG->EXTICR[position >> 2U];
        CLEAR_BIT(temp, ((uint32_t)0x0FU) << (4U * (position & 0x03U)));
        SET_BIT(temp, (GPIO_GET_INDEX(GPIOx)) << (4 * (position & 0x03U)));
        SYSCFG->EXTICR[position >> 2U] = temp;

        /* Clear EXTI line configuration */
        temp = EXTI->IMR;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & GPIO_MODE_IT) == GPIO_MODE_IT)
        {
          temp |= iocurrent;
        }
        EXTI->IMR = temp;

        temp = EXTI->EMR;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & GPIO_MODE_EVT) == GPIO_MODE_EVT)
        {
          temp |= iocurrent;
        }
        EXTI->EMR = temp;

        /* Clear Rising Falling edge configuration */
        temp = EXTI->RTSR;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & RISING_EDGE) == RISING_EDGE)
        {
          temp |= iocurrent;
        }
        EXTI->RTSR = temp;

        temp = EXTI->FTSR;
        temp &= ~((uint32_t)iocurrent);
        if((GPIO_Init->Mode & FALLING_EDGE) == FALLING_EDGE)
        {
          temp |= iocurrent;
        }
        EXTI->FTSR = temp;
      }
    }
    position++;
  }
}

/**
  * @brief  De-initializes the GPIOx peripheral registers to their default reset values.
  * @param  GPIOx: where x can be (A..E and H) to select the GPIO peripheral for STM32L0XX family devices.
  *                Note that GPIOE is not available on all devices.
  * @param  GPIO_Pin: specifies the port bit to be written.
  *                   This parameter can be one of GPIO_PIN_x where x can be (0..15).
  *                   All port bits are not necessarily available on all GPIOs.
  * @retval None
  */
void GpioDeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin)
{
  uint32_t position = 0x00U;
  uint32_t iocurrent = 0x00U;
  uint32_t tmp = 0x00U;

  /* Configure the port pins */
  while ((GPIO_Pin >> position) != 0)
  {
    /* Get the IO position */
    iocurrent = (GPIO_Pin) & (1U << position);

    if(iocurrent)
    {
      /*------------------------- GPIO Mode Configuration --------------------*/
      /* Configure IO Direction in Input Floting Mode */
      GPIOx->MODER |= (GPIO_MODER_MODE0 << (position * 2U));

      /* Configure the default Alternate Function in current IO */
      GPIOx->AFR[position >> 3U] &= ~((uint32_t)0xFU << ((uint32_t)(position & (uint32_t)0x07U) * 4U)) ;

      /* Configure the default value for IO Speed */
      GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEED0 << (position * 2U));

      /* Configure the default value IO Output Type */
      GPIOx->OTYPER  &= ~(GPIO_OTYPER_OT_0 << position) ;

      /* Deactivate the Pull-up oand Pull-down resistor for the current IO */
      GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPD0 << (position * 2U));

      /*------------------------- EXTI Mode Configuration --------------------*/
      /* Clear the External Interrupt or Event for the current IO */

      tmp = SYSCFG->EXTICR[position >> 2U];
      tmp &= (((uint32_t)0x0FU) << (4U * (position & 0x03U)));
      if(tmp == (GPIO_GET_INDEX(GPIOx) << (4U * (position & 0x03U))))
      {
        tmp = ((uint32_t)0x0FU) << (4U * (position & 0x03U));
        SYSCFG->EXTICR[position >> 2U] &= ~tmp;

        /* Clear EXTI line configuration */
        EXTI->IMR &= ~((uint32_t)iocurrent);
        EXTI->EMR &= ~((uint32_t)iocurrent);

        /* Clear Rising Falling edge configuration */
        EXTI->RTSR &= ~((uint32_t)iocurrent);
        EXTI->FTSR &= ~((uint32_t)iocurrent);
      }
    }
     position++;
  }
}

void setInput(GPIO_TypeDef  *port, uint32_t pin, uint32_t mode) {
  GpioInit_t GPIO_InitStruct;

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = mode;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GpioInit(port, &GPIO_InitStruct);
}  // set to input, which allows it to be pulled high by pullups

void setOutput(GPIO_TypeDef  *port, uint32_t pin) {
  GpioInit_t GPIO_InitStruct;

  GPIO_InitStruct.Pin = pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GpioInit(port, &GPIO_InitStruct);
}  // set to output

/**
* @brief This function handles EXTI line 0 to 1 interrupts.
*/
void EXTI0_1_IRQHandler(void)
{
  GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

/**
* @brief This function handles EXTI line 2 to 3 interrupts.
*/
void EXTI2_3_IRQHandler(void)
{
  GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

/**
  * @brief  Checks whether the specified EXTI line is asserted or not.
  * @param  __EXTI_LINE__: specifies the EXTI line to check.
  *          This parameter can be GPIO_PIN_x where x can be(0..15)
  * @retval The new state of __EXTI_LINE__ (SET or RESET).
  */
#define GPIO_EXTI_GET_IT(__EXTI_LINE__) (EXTI->PR & (__EXTI_LINE__))

/**
  * @brief  Clears the EXTI's line pending bits.
  * @param  __EXTI_LINE__: specifies the EXTI lines to clear.
  *          This parameter can be any combination of GPIO_PIN_x where x can be (0..15)
  * @retval None
  */
#define GPIO_EXTI_CLEAR_IT(__EXTI_LINE__) (EXTI->PR = (__EXTI_LINE__))

/**
  * @brief  This function handles EXTI interrupt request.
  * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
  * @retval None
  */
void GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin)
{
  /* EXTI line interrupt detected */
  if(GPIO_EXTI_GET_IT(GPIO_Pin) != RESET)
  {
    GPIO_EXTI_CLEAR_IT(GPIO_Pin);
    GPIO_EXTI_Callback(GPIO_Pin);
  }
}

/**
  * @brief  EXTI line detection callbacks.
  * @param  GPIO_Pin: Specifies the pins connected to the EXTI line.
  * @retval None
  */
__weak void GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);

  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
}
