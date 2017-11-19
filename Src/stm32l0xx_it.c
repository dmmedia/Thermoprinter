/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
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
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx.h"
#include "stm32l0xx_it.h"
#include "serial.h"

/* USER CODE BEGIN 0 */
extern uint8_t rx_buf[RXBUF_LEN], tx_buf[TXBUF_LEN];
extern volatile uint16_t rx_i, tx_o;
extern uint16_t tx_i;
extern volatile uint8_t tx_busy;

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M0+ Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line 0 to 1 interrupts.
*/
void EXTI0_1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_1_IRQn 0 */

  /* USER CODE END EXTI0_1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI0_1_IRQn 1 */

  /* USER CODE END EXTI0_1_IRQn 1 */
}

/**
* @brief This function handles EXTI line 2 to 3 interrupts.
*/
void EXTI2_3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_3_IRQn 0 */

  /* USER CODE END EXTI2_3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_3_IRQn 1 */

  /* USER CODE END EXTI2_3_IRQn 1 */
}

/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */

  /* USER CODE END EXTI4_15_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */

  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
    if((__HAL_UART_GET_IT(&huart2, UART_IT_RXNE) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_RXNE) != RESET))
    {
        rx_buf[rx_i & RXBUF_MSK] = (uint8_t)(huart2.Instance->RDR & 0x00FF);
        rx_i++;
        /* Clear RXNE interrupt flag */
        __HAL_UART_SEND_REQ(&huart2, UART_RXDATA_FLUSH_REQUEST);
    }
    if((__HAL_UART_GET_IT(&huart2, UART_IT_TXE) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TXE) != RESET))
    {
        if (tx_i == tx_o) {
            __HAL_UART_DISABLE_IT(&huart2, UART_IT_TXE);
            __HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);
        } else {
            huart2.Instance->TDR = (uint8_t)(tx_buf[TXBUF_MSK & tx_o] & (uint8_t)0xFF);
            tx_o++;
        }
    }
    if((__HAL_UART_GET_IT(&huart2, UART_IT_TC) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TC) != RESET))
    {
        tx_busy = 0;
        __HAL_UART_DISABLE_IT(&huart2, UART_IT_TC);
    }
    /* And never call default handler */
    return;

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles AES, RNG and LPUART1 interrupts / LPUART1 wake-up interrupt through EXTI line 28.
*/
void AES_RNG_LPUART1_IRQHandler(void)
{
  /* USER CODE BEGIN AES_RNG_LPUART1_IRQn 0 */
    if((__HAL_UART_GET_IT(&hlpuart1, UART_IT_RXNE) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(&hlpuart1, UART_IT_RXNE) != RESET))
    {
    	// read only of no errors, else skip byte
    	if (
    			(__HAL_UART_GET_IT(&hlpuart1, UART_IT_NE) == RESET) &&
				(__HAL_UART_GET_IT(&hlpuart1, UART_IT_FE) == RESET) &&
				(__HAL_UART_GET_IT(&hlpuart1, UART_IT_PE) == RESET)
		) {
    		store_char((uint8_t)(hlpuart1.Instance->RDR & 0x00FF));
    	} else {
    		hlpuart1.Instance->RDR;
    	}
        /* Clear RXNE interrupt flag */
        __HAL_UART_SEND_REQ(&hlpuart1, UART_RXDATA_FLUSH_REQUEST);
    }
    if((__HAL_UART_GET_IT(&hlpuart1, UART_IT_TXE) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(&hlpuart1, UART_IT_TXE) != RESET))
    {
        if (tx_i == tx_o) {
            __HAL_UART_DISABLE_IT(&hlpuart1, UART_IT_TXE);
            __HAL_UART_ENABLE_IT(&hlpuart1, UART_IT_TC);
        } else {
            hlpuart1.Instance->TDR = (uint8_t)(tx_buf[TXBUF_MSK & tx_o] & (uint8_t)0xFF);
            tx_o++;
        }
    }
    if((__HAL_UART_GET_IT(&hlpuart1, UART_IT_TC) != RESET) &&
       (__HAL_UART_GET_IT_SOURCE(&hlpuart1, UART_IT_TC) != RESET))
    {
        tx_busy = 0;
        __HAL_UART_DISABLE_IT(&hlpuart1, UART_IT_TC);
    }
    /* And never call default handler */
    return;

  /* USER CODE END AES_RNG_LPUART1_IRQn 0 */
  HAL_UART_IRQHandler(&hlpuart1);
  /* USER CODE BEGIN AES_RNG_LPUART1_IRQn 1 */

  /* USER CODE END AES_RNG_LPUART1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
