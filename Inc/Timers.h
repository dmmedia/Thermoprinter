#pragma once

/*
 * Timers.h
 *
 *  Created on: 8. jaan 2018
 *      Author: Den
 */

namespace Timers {
	//
	// Public definitions and constants
	//

	//
	// @brief  TIM Time base Configuration Structure definition
	//
	typedef struct
	{
	  uint32_t Prescaler;         //!< Specifies the prescaler value used to divide the TIM clock.
	  	  	  	  	  	  	  	  //   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF

	  uint32_t CounterMode;       //!< Specifies the counter mode.
								  //   This parameter can be a value of @ref TIM_Counter_Mode

	  uint32_t Period;            //!< Specifies the period value to be loaded into the active
								  //   Auto-Reload Register at the next update event.
								  //   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.

	  uint32_t ClockDivision;     //!< Specifies the clock division.
								  //   This parameter can be a value of @ref TIM_ClockDivision
	} TIM_Base_InitTypeDef;

	//
	// @brief  HAL Active channel structures definition
	//
	typedef enum
	{
	  TIM_ACTIVE_CHANNEL_1        = 0x01U,    //!< The active channel is 1
	  TIM_ACTIVE_CHANNEL_2        = 0x02U,    //!< The active channel is 2
	  TIM_ACTIVE_CHANNEL_3        = 0x04U,    //!< The active channel is 3
	  TIM_ACTIVE_CHANNEL_4        = 0x08U,    //!< The active channel is 4
	  TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U     //!< All active channels cleared
	} TIM_ActiveChannel;

	//
	// @brief  State structures definition
	//
	typedef enum
	{
	  TIM_STATE_RESET             = 0x00U,    //!< Peripheral not yet initialized or disabled
	  TIM_STATE_READY             = 0x01U,    //!< Peripheral Initialized and ready for use
	  TIM_STATE_BUSY              = 0x02U,    //!< An internal process is ongoing
	  TIM_STATE_TIMEOUT           = 0x03U,    //!< Timeout state
	  TIM_STATE_ERROR             = 0x04U     //!< Reception process is ongoing
	} TIM_StateTypeDef;

	//
	// @brief  TIM Time Base Handle Structure definition
	//
	typedef struct
	{
	  TIM_TypeDef              *Instance;     //!< Register base address
	  TIM_Base_InitTypeDef     Init;          //!< TIM Time Base required parameters
	  TIM_ActiveChannel    Channel;       //!< Active channel
	  DMA_HandleTypeDef        *hdma[7];      //!< DMA Handlers array
	                                          //   This array is accessed by a @ref DMA_Handle_index
	  HandleLockTypeDef          Lock;          //!< Locking object
	__IO TIM_StateTypeDef  State;         //!< TIM operation state
	} TIM_HandleTypeDef;

	#define TIM_ENABLE_IT(__HANDLE__, __INTERRUPT__)    ((__HANDLE__)->Instance->DIER |= (__INTERRUPT__))

	//
	// @brief  Sets the TIM Counter Register value on runtime.
	// @param  __HANDLE__ : TIM handle.
	// @param  __COUNTER__: specifies the Counter register new value.
	// @retval None
	//
	#define TIM_SET_COUNTER(__HANDLE__, __COUNTER__)  ((__HANDLE__)->Instance->CNT = (__COUNTER__))

	#define TIM_RESET_COUNTER(__HANDLE__) TIM_SET_COUNTER(__HANDLE__, 0)

	//
	// @brief  Gets the TIM Counter Register value on runtime.
	// @param  __HANDLE__ : TIM handle.
	// @retval None
	//
	#define TIM_GET_COUNTER(__HANDLE__) ((__HANDLE__)->Instance->CNT)

	//
	// @brief  Sets the TIM Autoreload Register value on runtime without calling
	//         another time any Init function.
	// @param  __HANDLE__ : TIM handle.
	// @param  __AUTORELOAD__: specifies the Counter register new value.
	// @retval None
	//
	#define TIM_SET_AUTORELOAD(__HANDLE__, __AUTORELOAD__) \
							do{                                                    \
								  (__HANDLE__)->Instance->ARR = (__AUTORELOAD__);  \
								  (__HANDLE__)->Init.Period = (__AUTORELOAD__);    \
							  } while(0)

	//
	// @brief  Gets the TIM Autoreload Register value on runtime
	// @param  __HANDLE__ : TIM handle.
	// @retval None
	//
	#define TIM_GET_AUTORELOAD(__HANDLE__) ((__HANDLE__)->Instance->ARR)

	// @defgroup TIM_Counter_Mode Counter mode
	// @{
	//
	#define TIM_COUNTERMODE_UP                 ((uint32_t)0x0000U)
	#define TIM_COUNTERMODE_DOWN               TIM_CR1_DIR
	#define TIM_COUNTERMODE_CENTERALIGNED1     TIM_CR1_CMS_0
	#define TIM_COUNTERMODE_CENTERALIGNED2     TIM_CR1_CMS_1
	#define TIM_COUNTERMODE_CENTERALIGNED3     TIM_CR1_CMS

	// @defgroup TIM_ClockDivision Clock division
	// @{
	//
	#define TIM_CLOCKDIVISION_DIV1                       ((uint32_t)0x0000U)
	#define TIM_CLOCKDIVISION_DIV2                       (TIM_CR1_CKD_0)
	#define TIM_CLOCKDIVISION_DIV4                       (TIM_CR1_CKD_1)

	//
	// Public variables
	//

	//
	// Public functions
	//

	//
	// @brief  Initializes the TIM Time base Unit according to the specified
	//         parameters in the TIM_HandleTypeDef and create the associated handle.
	// @param  htim : TIM handle
	// @retval HAL status
	//
	StatusTypeDef TIM_Base_Init(TIM_HandleTypeDef *htim);

	//
	// @brief This function configures the source of the time base.
	//        The time source is configured  to have 1ms time base with a dedicated
	//        Tick interrupt priority.
	// @note This function is called  automatically at the beginning of program after
	//       reset by Init() or at any time when clock is reconfigured  by RCC_ClockConfig().
	// @note In the default implementation, SysTick timer is the source of time base.
	//       It is used to generate interrupts at regular time intervals.
	//       Care must be taken if Delay() is called from a peripheral ISR process,
	//       The the SysTick interrupt must have higher priority (numerically lower)
	//       than the peripheral interrupt. Otherwise the caller ISR process will be blocked.
	//       The function is declared as __Weak  to be overwritten  in case of other
	//       implementation  in user file.
	// @param TickPriority: Tick interrupt priority.
	//
	void InitTick(uint32_t TickPriority);

	void Delay(__IO uint32_t delay_ms);

	//
	// @brief Provides a tick value in millisecond.
	// @retval tick value
	//
	uint32_t GetTick(void);

	// End

}

extern "C" {
	void SysTick_Handler(void);
}
