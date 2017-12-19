/*
 * tim.h
 *
 *  Created on: 17. dets 2017
 *      Author: Den
 */

#ifndef TIM_H_
#define TIM_H_

#define TIM_ENABLE_IT(__HANDLE__, __INTERRUPT__)    ((__HANDLE__)->Instance->DIER |= (__INTERRUPT__))
#define TIM_DISABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->DIER &= ~(__INTERRUPT__))

/**
  * @brief  Sets the TIM Counter Register value on runtime.
  * @param  __HANDLE__ : TIM handle.
  * @param  __COUNTER__: specifies the Counter register new value.
  * @retval None
  */
#define TIM_SET_COUNTER(__HANDLE__, __COUNTER__)  ((__HANDLE__)->Instance->CNT = (__COUNTER__))

#define TIM_RESET_COUNTER(__HANDLE__) TIM_SET_COUNTER(__HANDLE__, 0)

/**
  * @brief  Gets the TIM Counter Register value on runtime.
  * @param  __HANDLE__ : TIM handle.
  * @retval None
  */
#define TIM_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNT)

/**
  * @brief  Time Base configuration
  * @param  TIMx : TIM peripheral
  * @param   Structure : TIM Base configuration structure
  * @retval None
  */
static void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure)
{
  uint32_t tmpcr1 = 0U;
  tmpcr1 = TIMx->CR1;

  /* Set TIM Time Base Unit parameters ---------------------------------------*/
  if(IS_TIM_CC1_INSTANCE(TIMx) != RESET)
  {
    /* Select the Counter Mode */
    tmpcr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
    tmpcr1 |= Structure->CounterMode;
  }

  if(IS_TIM_CC1_INSTANCE(TIMx) != RESET)
  {
    /* Set the clock division */
    tmpcr1 &= ~TIM_CR1_CKD;
    tmpcr1 |= (uint32_t)Structure->ClockDivision;
  }

  TIMx->CR1 = tmpcr1;

  /* Set the Autoreload value */
  TIMx->ARR = (uint32_t)Structure->Period ;

  /* Set the Prescaler value */
  TIMx->PSC = (uint32_t)Structure->Prescaler;

  /* Generate an update event to reload the Prescaler value immediatly */
  TIMx->EGR = TIM_EGR_UG;
}

/**
  * @brief  Initializes the TIM Time base Unit according to the specified
  *         parameters in the TIM_HandleTypeDef and create the associated handle.
  * @param  htim : TIM handle
  * @retval HAL status
  */
StatusTypeDef TIM_Base_Init(TIM_HandleTypeDef *htim);

#endif /* TIM_H_ */
