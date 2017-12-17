/*
 * tim.cpp
 *
 *  Created on: 17. dets 2017
 *      Author: Den
 */

#include <stm32l0xx.h>
#include "typedefs.h"
#include "tim.h"
#include "stddef.h"

/**
  * @brief  Initializes the TIM Time base Unit according to the specified
  *         parameters in the TIM_HandleTypeDef and create the associated handle.
  * @param  htim : TIM handle
  * @retval HAL status
  */
StatusTypeDef TIM_Base_Init(TIM_HandleTypeDef *htim)
{
  /* Check the TIM handle allocation */
  if(htim == NULL)
  {
    return STATUS_ERROR;
  }

  if(htim->State == TIM_STATE_RESET)
  {
    /* Allocate lock resource and initialize it */
    htim->Lock = HANDLE_UNLOCKED;
  }

  /* Set the TIM state */
  htim->State= TIM_STATE_BUSY;

  /* Set the Time Base configuration */
  TIM_Base_SetConfig(htim->Instance, &htim->Init);

  /* Initialize the TIM state*/
  htim->State= TIM_STATE_READY;

  return STATUS_OK;
}
