/*
 * types.h
 *
 *  Created on: 17. dets 2017
 *      Author: Den
 */

#ifndef TYPEDEFS_H_
#define TYPEDEFS_H_

#include <stdint.h>

typedef enum
{
  HANDLE_UNLOCKED = 0x00U,
  HANDLE_LOCKED   = 0x01U
} HandleLockTypeDef;

/**
  * @brief  Status structures definition
  */
typedef enum
{
  STATUS_OK       = 0x00U,
  STATUS_ERROR    = 0x01U,
  STATUS_BUSY     = 0x02U,
  STATUS_TIMEOUT  = 0x03U
} StatusTypeDef;

/**
  * @brief  TIM Time base Configuration Structure definition
  */
typedef struct
{
  uint32_t Prescaler;         /*!< Specifies the prescaler value used to divide the TIM clock.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

  uint32_t CounterMode;       /*!< Specifies the counter mode.
                                   This parameter can be a value of @ref TIM_Counter_Mode */

  uint32_t Period;            /*!< Specifies the period value to be loaded into the active
                                   Auto-Reload Register at the next update event.
                                   This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF.  */

  uint32_t ClockDivision;     /*!< Specifies the clock division.
                                   This parameter can be a value of @ref TIM_ClockDivision */
} TIM_Base_InitTypeDef;

/**
  * @brief  HAL Active channel structures definition
  */
typedef enum
{
  TIM_ACTIVE_CHANNEL_1        = 0x01U,    /*!< The active channel is 1     */
  TIM_ACTIVE_CHANNEL_2        = 0x02U,    /*!< The active channel is 2     */
  TIM_ACTIVE_CHANNEL_3        = 0x04U,    /*!< The active channel is 3     */
  TIM_ACTIVE_CHANNEL_4        = 0x08U,    /*!< The active channel is 4     */
  TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U     /*!< All active channels cleared */
} TIM_ActiveChannel;

/**
  * @brief  DMA Configuration Structure definition
  */
typedef struct
{
  uint32_t Request;                   /*!< Specifies the request selected for the specified channel.
                                           This parameter can be a value of @ref DMA_request */

  uint32_t Direction;                 /*!< Specifies if the data will be transferred from memory to peripheral,
                                           from memory to memory or from peripheral to memory.
                                           This parameter can be a value of @ref DMA_Data_transfer_direction */

  uint32_t PeriphInc;                 /*!< Specifies whether the Peripheral address register should be incremented or not.
                                           When Memory to Memory transfer is used, this is the Source Increment mode
                                           This parameter can be a value of @ref DMA_Peripheral_incremented_mode */

  uint32_t MemInc;                    /*!< Specifies whether the memory address register should be incremented or not.
                                           When Memory to Memory transfer is used, this is the Destination Increment mode
                                           This parameter can be a value of @ref DMA_Memory_incremented_mode */

  uint32_t PeriphDataAlignment;       /*!< Specifies the Peripheral data width.
                                           When Memory to Memory transfer is used, this is the Source Alignment format
                                           This parameter can be a value of @ref DMA_Peripheral_data_size */

  uint32_t MemDataAlignment;          /*!< Specifies the Memory data width.
                                           When Memory to Memory transfer is used, this is the Destination Alignment format
                                           This parameter can be a value of @ref DMA_Memory_data_size */

  uint32_t Mode;                      /*!< Specifies the operation mode of the DMAy Channelx (Normal or Circular).
                                           This parameter can be a value of @ref DMA_mode
                                           @note The circular buffer mode cannot be used if the memory-to-memory
                                                 data transfer is configured on the selected Channel */

  uint32_t Priority;                   /*!< Specifies the software priority for the DMAy Channelx.
                                            This parameter can be a value of @ref DMA_Priority_level */
} DMA_InitTypeDef;

/**
  * @brief  DMA State structures definition
  */
typedef enum
{
  DMA_STATE_RESET             = 0x00U,  /*!< DMA not yet initialized or disabled */
  DMA_STATE_READY             = 0x01U,  /*!< DMA process success and ready for use   */
  DMA_STATE_BUSY              = 0x02U,  /*!< DMA process is ongoing              */
  DMA_STATE_TIMEOUT           = 0x03U,  /*!< DMA timeout state                   */
  DMA_STATE_ERROR             = 0x04U,  /*!< DMA error state                     */
  DMA_STATE_READY_HALF        = 0x05U,  /*!< DMA Half process success            */
} DMA_StateTypeDef;

/**
  * @brief  DMA handle Structure definition
  */
typedef struct __DMA_HandleTypeDef
{
  DMA_Channel_TypeDef    *Instance;                                                   /*!< Register base address                  */

  DMA_InitTypeDef       Init;                                                         /*!< DMA communication parameters           */

  HandleLockTypeDef       Lock;                                                         /*!< DMA locking object                     */

  __IO DMA_StateTypeDef  State;                                                   /*!< DMA transfer state                     */

  void                  *Parent;                                                      /*!< Parent object state                    */

  void                  (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);     /*!< DMA transfer complete callback         */

  void                  (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma); /*!< DMA Half transfer complete callback    */

  void                  (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);    /*!< DMA transfer error callback            */

  void                  (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);    /*!< DMA transfer abort callback          */

__IO uint32_t          ErrorCode;                                                     /*!< DMA Error code                         */

} DMA_HandleTypeDef;

/**
  * @brief  State structures definition
  */
typedef enum
{
  TIM_STATE_RESET             = 0x00U,    /*!< Peripheral not yet initialized or disabled  */
  TIM_STATE_READY             = 0x01U,    /*!< Peripheral Initialized and ready for use    */
  TIM_STATE_BUSY              = 0x02U,    /*!< An internal process is ongoing              */
  TIM_STATE_TIMEOUT           = 0x03U,    /*!< Timeout state                               */
  TIM_STATE_ERROR             = 0x04U     /*!< Reception process is ongoing                */
} TIM_StateTypeDef;

/**
  * @brief  TIM Time Base Handle Structure definition
  */
typedef struct
{
  TIM_TypeDef              *Instance;     /*!< Register base address             */
  TIM_Base_InitTypeDef     Init;          /*!< TIM Time Base required parameters */
  TIM_ActiveChannel    Channel;       /*!< Active channel                    */
  DMA_HandleTypeDef        *hdma[7];      /*!< DMA Handlers array
                                             This array is accessed by a @ref DMA_Handle_index */
  HandleLockTypeDef          Lock;          /*!< Locking object                    */
__IO TIM_StateTypeDef  State;         /*!< TIM operation state               */
} TIM_HandleTypeDef;

typedef float float32_t;
typedef double float64_t;

#endif /* TYPEDEFS_H_ */
