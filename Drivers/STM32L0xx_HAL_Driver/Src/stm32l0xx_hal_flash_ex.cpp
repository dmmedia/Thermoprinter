/**
  ******************************************************************************
  * @file    stm32l0xx_hal_flash_ex.c
  * @author  MCD Application Team
  * @version V1.8.1
  * @date    14-April-2017
  * @brief   Extended FLASH HAL module driver.
  *    
  *          This file provides firmware functions to manage the following 
  *          functionalities of the internal FLASH memory:
  *            + FLASH Interface configuration
  *            + FLASH Memory Erasing
  *            + DATA EEPROM Programming/Erasing
  *            + Option Bytes Programming
  *            + Interrupts management
  *    
  @verbatim
  ==============================================================================
               ##### Flash peripheral Extended features  #####
  ==============================================================================
           
  [..] Comparing to other products, the FLASH interface for STM32L0xx
       devices contains the following additional features        
       (+) Erase functions
       (+) DATA_EEPROM memory management
       (+) BOOT option bit configuration       
       (+) PCROP protection for all sectors
   
                      ##### How to use this driver #####
  ==============================================================================
  [..] This driver provides functions to configure and program the FLASH memory 
       of all STM32L0xx. It includes:
       (+) Full DATA_EEPROM erase and program management
       (+) Boot activation
       (+) PCROP protection configuration and control for all pages
  
  @endverbatim
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
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

/** @addtogroup STM32L0xx_HAL_Driver
  * @{
  */
#ifdef HAL_FLASH_MODULE_ENABLED

/** @addtogroup FLASH
  * @{
  */
/** @addtogroup FLASH_Private_Variables
 * @{
 */
/* Variables used for Erase pages under interruption*/
extern FLASH_ProcessTypeDef pFlash;
/**
  * @}
  */

/**
  * @}
  */
  
/** @defgroup FLASHEx FLASHEx
  * @brief FLASH HAL Extension module driver
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/** @defgroup FLASHEx_Private_Constants FLASHEx Private Constants
 * @{
 */
/**
  * @}
  */

/* Private macro -------------------------------------------------------------*/
/** @defgroup FLASHEx_Private_Macros FLASHEx Private Macros
  * @{
  */
/**
  * @}
  */ 

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/** @defgroup FLASHEx_Private_Functions FLASHEx Private Functions
 * @{
 */
void                      FLASH_PageErase(uint32_t PageAddress);
static uint8_t            FLASH_OB_GetRDP(void);
static uint8_t            FLASH_OB_GetUser(void);
static uint8_t            FLASH_OB_GetBOR(void);
static uint8_t            FLASH_OB_GetBOOTBit1(void);
static uint32_t           FLASH_OB_GetWRP(void);

/**
  * @}
  */

/* Exported functions ---------------------------------------------------------*/
/** @defgroup FLASHEx_Exported_Functions_Group2 Option Bytes Programming functions
 *  @brief   Option Bytes Programming functions
 *
@verbatim   
  ==============================================================================
                ##### Option Bytes Programming functions ##### 
  ==============================================================================  

    [..] Any operation of erase or program should follow these steps:
    (#) Call the @ref HAL_FLASH_OB_Unlock() function to enable the Flash option control 
        register access.
    (#) Call following function to program the desired option bytes.
        (++) @ref HAL_FLASHEx_OBProgram:
         - To Enable/Disable the desired sector write protection.
         - To set the desired read Protection Level.
         - To configure the user option Bytes: IWDG, STOP and the Standby.
         - To Set the BOR level.
    (#) Once all needed option bytes to be programmed are correctly written, call the
        @ref HAL_FLASH_OB_Launch(void) function to launch the Option Bytes programming process.
    (#) Call the @ref HAL_FLASH_OB_Lock() to disable the Flash option control register access (recommended
        to protect the option Bytes against possible unwanted operations).

    [..] Proprietary code Read Out Protection (PcROP):
    (#) The PcROP sector is selected by using the same option bytes as the Write
        protection (nWRPi bits). As a result, these 2 options are exclusive each other.
    (#) In order to activate the PcROP (change the function of the nWRPi option bits), 
        the WPRMOD option bit must be activated.
    (#) The active value of nWRPi bits is inverted when PCROP mode is active, this
        means: if WPRMOD = 1 and nWRPi = 1 (default value), then the user sector "i"
        is read/write protected.
    (#) To activate PCROP mode for Flash sector(s), you need to call the following function:
        (++) @ref HAL_FLASHEx_AdvOBProgram in selecting sectors to be read/write protected
        (++) @ref HAL_FLASHEx_OB_SelectPCROP to enable the read/write protection

@endverbatim
  * @{
  */

/**
  * @brief   Get the Option byte configuration
  * @param  pOBInit pointer to an FLASH_OBInitStruct structure that
  *         contains the configuration information for the programming.
  * 
  * @retval None
  */
void HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit)
{
  pOBInit->OptionType = OPTIONBYTE_WRP | OPTIONBYTE_RDP | OPTIONBYTE_USER | OPTIONBYTE_BOR;

  /* Get WRP sector */
  pOBInit->WRPSector = FLASH_OB_GetWRP();

  /*Get RDP Level*/
  pOBInit->RDPLevel   = FLASH_OB_GetRDP();

  /*Get USER*/
  pOBInit->USERConfig = FLASH_OB_GetUser();

  /*Get BOR Level*/
  pOBInit->BORLevel   = FLASH_OB_GetBOR();

  /* Get BOOT bit 1 config OB */
  pOBInit->BOOTBit1Config = FLASH_OB_GetBOOTBit1();
}

#if defined(FLASH_OPTR_WPRMOD) || defined(FLASH_OPTR_BFB2)
    
/**
  * @brief  Get the OBEX byte configuration
  * @param  pAdvOBInit pointer to an FLASH_AdvOBProgramInitTypeDef structure that
  *         contains the configuration information for the programming.
  * 
  * @retval None
  */
void HAL_FLASHEx_AdvOBGetConfig(FLASH_AdvOBProgramInitTypeDef *pAdvOBInit)
{
  pAdvOBInit->OptionType = 0;
  
#if defined(FLASH_OPTR_WPRMOD)
      
  pAdvOBInit->OptionType |= OPTIONBYTE_PCROP;


  /* Get PCROP state */
  pAdvOBInit->PCROPState = (FLASH->OPTR & FLASH_OPTR_WPRMOD) >> FLASH_OPTR_WPRMOD_Pos;
  /* Get PCROP protected sector */
  pAdvOBInit->PCROPSector = FLASH->WRPR;

#endif /* FLASH_OPTR_WPRMOD */

}

#endif /* FLASH_OPTR_WPRMOD || FLASH_OPTR_BFB2 */

/**
  * @}
  */

/** @defgroup FLASHEx_Exported_Functions_Group3 DATA EEPROM Programming functions
 *  @brief   DATA EEPROM Programming functions
 *
@verbatim   
 ===============================================================================
                     ##### DATA EEPROM Programming functions ##### 
 ===============================================================================  
 
    [..] Any operation of erase or program should follow these steps:
    (#) Call the @ref HAL_FLASHEx_DATAEEPROM_Unlock() function to enable the data EEPROM access
        and Flash program erase control register access.
    (#) Call the desired function to erase or program data.
    (#) Call the @ref HAL_FLASHEx_DATAEEPROM_Lock() to disable the data EEPROM access
        and Flash program erase control register access(recommended
        to protect the DATA_EEPROM against possible unwanted operation).

@endverbatim
  * @{
  */

/**
  * @brief  Unlocks the data memory and FLASH_PECR register access.
  * @retval HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef HAL_FLASHEx_DATAEEPROM_Unlock(void)
{
  if((FLASH->PECR & FLASH_PECR_PELOCK) != RESET)
  {  
    /* Unlocking the Data memory and FLASH_PECR register access*/
    FLASH->PEKEYR = FLASH_PEKEY1;
    FLASH->PEKEYR = FLASH_PEKEY2;
  }
  else
  {
    return HAL_ERROR;
  }
  return HAL_OK;  
}

/**
  * @brief  Locks the Data memory and FLASH_PECR register access.
  * @retval HAL_StatusTypeDef HAL Status
  */
HAL_StatusTypeDef HAL_FLASHEx_DATAEEPROM_Lock(void)
{
  /* Set the PELOCK Bit to lock the data memory and FLASH_PECR register access */
  SET_BIT(FLASH->PECR, FLASH_PECR_PELOCK);
  
  return HAL_OK;
}

/**
  * @brief  Enable DATA EEPROM fixed Time programming (2*Tprog).
  * @retval None
  */
void HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram(void)
{
  SET_BIT(FLASH->PECR, FLASH_PECR_FIX);
}

/**
  * @brief  Disables DATA EEPROM fixed Time programming (2*Tprog).
  * @retval None
  */
void HAL_FLASHEx_DATAEEPROM_DisableFixedTimeProgram(void)
{
  CLEAR_BIT(FLASH->PECR, FLASH_PECR_FIX);
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup FLASHEx_Private_Functions
 * @{
 */

/*
==============================================================================
              OPTIONS BYTES
==============================================================================
*/
/**
  * @brief  Returns the FLASH User Option Bytes values.
  * @retval The FLASH User Option Bytes.
  */
static uint8_t FLASH_OB_GetUser(void)
{
  /* Return the User Option Byte */
  return (uint8_t)((FLASH->OPTR & FLASH_OPTR_USER) >> 16U);
}

/**
  * @brief  Returns the FLASH Read Protection level.
  * @retval FLASH RDP level
  *         This parameter can be one of the following values:
  *            @arg @ref OB_RDP_LEVEL_0 No protection
  *            @arg @ref OB_RDP_LEVEL_1 Read protection of the memory
  *            @arg @ref OB_RDP_LEVEL_2 Full chip protection
  */
static uint8_t FLASH_OB_GetRDP(void)
{
  return (uint8_t)(FLASH->OPTR & FLASH_OPTR_RDPROT);
}

/**
  * @brief  Returns the FLASH BOR level.
  * @retval The BOR level Option Bytes.
  */
static uint8_t FLASH_OB_GetBOR(void)
{
  /* Return the BOR level */
  return (uint8_t)((FLASH->OPTR & (uint32_t)FLASH_OPTR_BOR_LEV) >> 16U);
}

/**
  * @brief  Returns the FLASH BOOT bit1 value.
  * @retval The BOOT bit 1 value Option Bytes.
  */
static uint8_t FLASH_OB_GetBOOTBit1(void)
{
  /* Return the BOR level */
  return (FLASH->OPTR & FLASH_OPTR_BOOT1) >> FLASH_OPTR_BOOT1_Pos;

}

/**
  * @brief  Returns the FLASH Write Protection Option Bytes value.
  * @retval The FLASH Write Protection Option Bytes value.
  */
static uint32_t FLASH_OB_GetWRP(void)
{
  /* Return the FLASH write protection Register value */
  return (uint32_t)(FLASH->WRPR);
}

/**
  * @}
  */

/**
  * @}
  */

/** @addtogroup FLASH
  * @{
  */


/** @addtogroup FLASH_Private_Functions
 * @{
 */

/**
  * @brief  Erases a specified page in program memory.
  * @param  PageAddress The page address in program memory to be erased.
  * @note   A Page is erased in the Program memory only if the address to load 
  *         is the start address of a page (multiple of @ref FLASH_PAGE_SIZE bytes).
  * @retval None
  */
void FLASH_PageErase(uint32_t PageAddress)
{
  /* Clean the error context */
  pFlash.ErrorCode = HAL_FLASH_ERROR_NONE;

  /* Set the ERASE bit */
  SET_BIT(FLASH->PECR, FLASH_PECR_ERASE);

  /* Set PROG bit */
  SET_BIT(FLASH->PECR, FLASH_PECR_PROG);

  /* Write 00000000h to the first word of the program page to erase */
  *(__IO uint32_t *)(uint32_t)(PageAddress & ~(FLASH_PAGE_SIZE - 1)) = 0x00000000;
}
  
/**
  * @}
  */

/**
  * @}
  */

#endif /* HAL_FLASH_MODULE_ENABLED */
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
