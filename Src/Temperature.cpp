/*
 * Temperature.cpp
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

#include <stdint.h>
#include <stm32l0xx.h>
#include "macros.h"
#include "gpio.h"
#include "Configuration.h"
#include "typedefs.h"
#include "Conditionals.h"
#include "typedefs.h"
#include "main.h"
#include "Timers.h"
#include "Temperature.h"
#include "Planner.h"
#include "Stepper.h"
#include "Endstops.h"
#include "rcc.h"
#include "serial.h"
#include "Thermoprinter.h"

namespace AdcManager {
	//
	// Private definitions and constants
	//

	//
	// States for ADC reading in the ISR
	//
	enum ADCSensorState {
		PrepareSensors,
		MeasureSensors,
		SensorsReady, // Temperatures ready. Delay the next round of readings to let ADC pins settle.
		StartupDelay  // Startup, delay initial temp reading a tiny bit so the hardware can settle
	};

	//
	// @brief  ADC group regular oversampling structure definition
	//
	typedef struct
	{
	    uint32_t Ratio;                         //!< Configures the oversampling ratio.
												// This parameter can be a value of @ref ADC_Oversampling_Ratio

	    uint32_t RightBitShift;                 //!< Configures the division coefficient for the Oversampler.
												// This parameter can be a value of @ref ADC_Right_Bit_Shift

	    uint32_t TriggeredMode;                 //!< Selects the regular triggered oversampling mode.
												// This parameter can be a value of @ref ADC_Triggered_Oversampling_Mode
	} ADC_OversamplingTypeDef;

	//
	// @brief  Structure definition of ADC instance and ADC group regular.
	// @note   Parameters of this structure are shared within 2 scopes:
	//          - Scope entire ADC (differentiation done for compatibility with some other STM32 series featuring ADC groups regular and injected): ClockPrescaler, Resolution, DataAlign,
	//            ScanConvMode, EOCSelection, LowPowerAutoWait.
	//          - Scope ADC group regular: ContinuousConvMode, NbrOfConversion, DiscontinuousConvMode,
	//            ExternalTrigConv, ExternalTrigConvEdge, DMAContinuousRequests, Overrun, OversamplingMode, Oversampling.
	// @note   The setting of these parameters by function HAL_ADC_Init() is conditioned to ADC state.
	//         ADC state can be either:
	//          - For all parameters: ADC disabled
	//          - For all parameters except 'ClockPrescaler' and 'Resolution': ADC enabled without conversion on going on group regular.
	//         If ADC is not in the appropriate state to modify some parameters, these parameters setting is bypassed
	//         without error reporting (as it can be the expected behavior in case of intended action to update another parameter
	//         (which fulfills the ADC state condition) on the fly).
	//
	typedef struct
	{
		uint32_t ClockPrescaler;        //!< Select ADC clock source (synchronous clock derived from APB clock or asynchronous clock derived from ADC dedicated HSI RC oscillator) and clock prescaler.
										//   This parameter can be a value of @ref ADC_ClockPrescaler.
									  	//   Note: In case of synchronous clock mode based on HCLK/1, the configuration must be enabled only
									  	//		 if the system clock has a 50% duty clock cycle (APB prescaler configured inside RCC
									  	//		 must be bypassed and PCLK clock must have 50% duty cycle). Refer to reference manual for details.
									  	//   Note: In case of usage of the ADC dedicated HSI RC oscillator, it must be preliminarily enabled at RCC top level.
									  	//   Note: This parameter can be modified only if the ADC is disabled.

		uint32_t Resolution;            //!< Configure the ADC resolution.
										//   This parameter can be a value of @ref ADC_Resolution

		uint32_t DataAlign;             //!< Specify ADC data alignment in conversion data register (right or left).
										//   Refer to reference manual for alignments formats versus resolutions.
								      	//   This parameter can be a value of @ref ADC_Data_align

		uint32_t ScanConvMode;          //!< Configure the sequencer of regular group.
										//   This parameter can be associated to parameter 'DiscontinuousConvMode' to have main sequence subdivided in successive parts.
									  	//   Sequencer is automatically enabled if several channels are set (sequencer cannot be disabled, as it can be the case on other STM32 devices):
									  	//   If only 1 channel is set: Conversion is performed in single mode.
									  	//   If several channels are set:  Conversions are performed in sequence mode (ranks defined by each channel number: channel 0 fixed on rank 0, channel 1 fixed on rank1, ...).
									  	//								 Scan direction can be set to forward (from channel 0 to channel 18) or backward (from channel 18 to channel 0).
									  	//   This parameter can be a value of @ref ADC_Scan_mode

		uint32_t EOCSelection;          //!< Specify which EOC (End Of Conversion) flag is used for conversion by polling and interruption: end of unitary conversion or end of sequence conversions.
										//   This parameter can be a value of @ref ADC_EOCSelection.

		uint32_t LowPowerAutoWait;      //!< Select the dynamic low power Auto Delay: new conversion start only when the previous
										//   conversion (for ADC group regular) has been retrieved by user software,
									  	//   using function HAL_ADC_GetValue().
									  	//   This feature automatically adapts the frequency of ADC conversions triggers to the speed of the system that reads the data. Moreover, this avoids risk of overrun
									  	//   for low frequency applications.
									  	//   This parameter can be set to ENABLE or DISABLE.
									  	//   Note: Do not use with interruption or DMA (HAL_ADC_Start_IT(), HAL_ADC_Start_DMA()) since they clear immediately the EOC flag
									  	//		 to free the IRQ vector sequencer.
									  	//		 Do use with polling: 1. Start conversion with HAL_ADC_Start(), 2. Later on, when ADC conversion data is needed:
    								  	//		 use HAL_ADC_PollForConversion() to ensure that conversion is completed and HAL_ADC_GetValue() to retrieve conversion result and trig another conversion start.

		uint32_t LowPowerAutoPowerOff;  //!< Select the auto-off mode: the ADC automatically powers-off after a conversion and automatically wakes-up when a new conversion is triggered (with startup time between trigger and start of sampling).
										//   This feature can be combined with automatic wait mode (parameter 'LowPowerAutoWait').
									  	//   This parameter can be set to ENABLE or DISABLE.
									  	//   Note: If enabled, this feature also turns off the ADC dedicated 14 MHz RC oscillator (HSI14)

		uint32_t ContinuousConvMode;    //!< Specify whether the conversion is performed in single mode (one conversion) or continuous mode for ADC group regular,
										//   after the first ADC conversion start trigger occurred (software start or external trigger).
									  	//   This parameter can be set to ENABLE or DISABLE.

		uint32_t DiscontinuousConvMode; //!< Specify whether the conversions sequence of ADC group regular is performed in Complete-sequence/Discontinuous-sequence
										//   (main sequence subdivided in successive parts).
									  	//   Discontinuous mode is used only if sequencer is enabled (parameter 'ScanConvMode'). If sequencer is disabled, this parameter is discarded.
									  	//   Discontinuous mode can be enabled only if continuous mode is disabled. If continuous mode is enabled, this parameter setting is discarded.
									  	//   This parameter can be set to ENABLE or DISABLE.
									  	//   Note: On this STM32 serie, ADC group regular number of discontinuous ranks increment is fixed to one-by-one.

		uint32_t ExternalTrigConv;      //!< Select the external event source used to trigger ADC group regular conversion start.
										//   If set to ADC_SOFTWARE_START, external triggers are disabled and software trigger is used instead.
									  	//   This parameter can be a value of @ref ADC_regular_external_trigger_source.
									  	//   Caution: external trigger source is common to all ADC instances.

		uint32_t ExternalTrigConvEdge;  //!< Select the external event edge used to trigger ADC group regular conversion start.
										//   If trigger source is set to ADC_SOFTWARE_START, this parameter is discarded.
									  	//   This parameter can be a value of @ref ADC_regular_external_trigger_edge

		uint32_t DMAContinuousRequests; //!< Specify whether the DMA requests are performed in one shot mode (DMA transfer stops when number of conversions is reached)
										//   or in continuous mode (DMA transfer unlimited, whatever number of conversions).
									  	//   This parameter can be set to ENABLE or DISABLE.
									  	//   Note: In continuous mode, DMA must be configured in circular mode. Otherwise an overrun will be triggered when DMA buffer maximum pointer is reached.

		uint32_t Overrun;               //!< Select the behavior in case of overrun: data overwritten or preserved (default).
										//   This parameter can be a value of @ref ADC_Overrun.
									  	//   Note: In case of overrun set to data preserved and usage with programming model with interruption (HAL_Start_IT()): ADC IRQ handler has to clear
									  	//   end of conversion flags, this induces the release of the preserved data. If needed, this data can be saved in function
									  	//   HAL_ADC_ConvCpltCallback(), placed in user program code (called before end of conversion flags clear).
									  	//   Note: Error reporting with respect to the conversion mode:
									  	//		 - Usage with ADC conversion by polling for event or interruption: Error is reported only if overrun is set to data preserved. If overrun is set to data
									  	//		   overwritten, user can willingly not read all the converted data, this is not considered as an erroneous case.
									  	//		 - Usage with ADC conversion by DMA: Error is reported whatever overrun setting (DMA is expected to process all data from data register).

		uint32_t LowPowerFrequencyMode; //!< When selecting an analog ADC clock frequency lower than 2.8MHz,
										//   it is mandatory to first enable the Low Frequency Mode.
									  	//   This parameter can be set to ENABLE or DISABLE.
									  	//   Note: This parameter can be modified only if there is no conversion is ongoing.


		uint32_t SamplingTime;                 //!< The sample time common to all channels.
											   //   Unit: ADC clock cycles
											   //   This parameter can be a value of @ref ADC_sampling_times
											   //   Note: This parameter can be modified only if there is no conversion ongoing.

		uint32_t OversamplingMode;              //!< Specify whether the oversampling feature is enabled or disabled.
												//   This parameter can be set to ENABLE or DISABLE.
											  	//   Note: This parameter can be modified only if there is no conversion is ongoing on ADC group regular.


		ADC_OversamplingTypeDef  Oversample;   //!< Specify the Oversampling parameters
											   //   Caution: this setting overwrites the previous oversampling configuration if oversampling is already enabled.
	} ADC_InitTypeDef;

	//
	// @brief  ADC handle Structure definition
	//
	typedef struct
	{
		ADC_TypeDef                   *Instance;              //!< Register base address

		ADC_InitTypeDef               Init;                   //!< ADC required parameters

		DMA_HandleTypeDef             *DMA_Handle;            //!< Pointer DMA Handler

		HandleLockTypeDef               Lock;                   //!< ADC locking object

		__IO uint32_t                 State;                  //!< ADC communication state (bitmap of ADC states)

		__IO uint32_t                 ErrorCode;              //!< ADC Error code
	} ADC_HandleTypeDef;

	//
	// @brief  Structure definition of ADC channel for regular group
	// @note   The setting of these parameters by function HAL_ADC_ConfigChannel() is conditioned to ADC state.
	//         ADC state can be either:
	//          - For all parameters: ADC disabled or enabled without conversion on going on regular group.
	//         If ADC is not in the appropriate state to modify some parameters, these parameters setting is bypassed
	//         without error reporting (as it can be the expected behavior in case of intended action to update another parameter (which fulfills the ADC state condition) on the fly).
	//
	typedef struct
	{
		uint32_t Channel;                //!< Specify the channel to configure into ADC regular group.
										 //   This parameter can be a value of @ref ADC_channels
										 //   Note: Depending on devices, some channels may not be available on device package pins. Refer to device datasheet for channels availability.

		uint32_t Rank;                   //!< Add or remove the channel from ADC regular group sequencer.
										 //   On STM32L0 devices,  number of ranks in the sequence is defined by number of channels enabled, rank of each channel is defined by channel number
										 //   (channel 0 fixed on rank 0, channel 1 fixed on rank1, ...).
										 //   Despite the channel rank is fixed, this parameter allow an additional possibility: to remove the selected rank (selected channel) from sequencer.
										 //   This parameter can be a value of @ref ADC_rank
	} ADC_ChannelConfTypeDef;

	#define OVERSAMPLENR 16

	// User-defined table 1
	// Dummy Thermistor table.. It will ALWAYS read a fixed value.
	#ifndef DUMMY_THERMISTOR_998_VALUE
		#define DUMMY_THERMISTOR_998_VALUE 25
	#endif

	// Set the high and low raw values for the print head
	// For thermistors the highest temperature results in the lowest ADC value
	#ifndef PRINTHEAD_RAW_HI_TEMP
	  	#define PRINTHEAD_RAW_HI_TEMP 0
		#define PRINTHEAD_RAW_LO_TEMP 16383
	#endif

	#define _TT_NAME(_N) temptable_ ## _N
	#define TT_NAME(_N) _TT_NAME(_N)

	#if defined(THERMISTORHEATER_0)
		#define PRINTHEAD_TEMPTABLE TT_NAME(THERMISTORHEATER_0)
		#define PRINTHEAD_TEMPTABLE_LEN COUNT(PRINTHEAD_TEMPTABLE)
	#elif defined(PRINTHEAD_USES_THERMISTOR)
		#error "No heater 0 thermistor table specified"
	#else
		#define PRINTHEAD_TEMPTABLE NULL
		#define PRINTHEAD_TEMPTABLE_LEN 0
	#endif

	// Set the high and low raw values for the battery
	#ifndef BATTERY_RAW_HI_VOLT
		#define BATTERY_RAW_HI_VOLT 16383
		#define BATTERY_RAW_LO_VOLT 0
	#endif

	#define BATTERY_VOLTTABLE TT_NAME(THERMISTORHEATER_0) // TODO: fixme
	#define BATTERY_VOLTTABLE_LEN COUNT(BATTERY_VOLTTABLE)

	// Minimum number of Temperature::ISR loops between sensor readings.
	// Multiplied by 16 (OVERSAMPLENR) to obtain the total time to
	// get all oversampled sensor readings
	#define MIN_ADC_ISR_LOOPS 10

	// ADC parameters
	#define ADCCONVERTEDVALUES_BUFFER_SIZE 2U    /* Size of array containing ADC converted values: set to ADC sequencer number of ranks converted, to have a rank in each address */

	// @defgroup ADC_channels ADC_Channels
	//
	#define ADC_CHANNEL_0           ((uint32_t)(ADC_CHSELR_CHSEL0))
	#define ADC_CHANNEL_1           ((uint32_t)(ADC_CHSELR_CHSEL1) | ADC_CFGR1_AWDCH_0)
	#define ADC_CHANNEL_2           ((uint32_t)(ADC_CHSELR_CHSEL2) | ADC_CFGR1_AWDCH_1)
	#define ADC_CHANNEL_3           ((uint32_t)(ADC_CHSELR_CHSEL3)| ADC_CFGR1_AWDCH_1 | ADC_CFGR1_AWDCH_0)
	#define ADC_CHANNEL_4           ((uint32_t)(ADC_CHSELR_CHSEL4)| ADC_CFGR1_AWDCH_2)
	#define ADC_CHANNEL_5           ((uint32_t)(ADC_CHSELR_CHSEL5)| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_0)
	#define ADC_CHANNEL_6           ((uint32_t)(ADC_CHSELR_CHSEL6)| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_1)
	#define ADC_CHANNEL_7           ((uint32_t)(ADC_CHSELR_CHSEL7)| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_1 | ADC_CFGR1_AWDCH_0)
	#define ADC_CHANNEL_8           ((uint32_t)(ADC_CHSELR_CHSEL8)| ADC_CFGR1_AWDCH_3)
	#define ADC_CHANNEL_9           ((uint32_t)(ADC_CHSELR_CHSEL9)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_0)
	#define ADC_CHANNEL_10          ((uint32_t)(ADC_CHSELR_CHSEL10)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_1)
	#define ADC_CHANNEL_11          ((uint32_t)(ADC_CHSELR_CHSEL11)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_1| ADC_CFGR1_AWDCH_0)
	#define ADC_CHANNEL_12          ((uint32_t)(ADC_CHSELR_CHSEL12)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_2)
	#define ADC_CHANNEL_13          ((uint32_t)(ADC_CHSELR_CHSEL13)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_0)
	#define ADC_CHANNEL_14          ((uint32_t)(ADC_CHSELR_CHSEL14)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_1)
	#define ADC_CHANNEL_15          ((uint32_t)(ADC_CHSELR_CHSEL15)| ADC_CFGR1_AWDCH_3| ADC_CFGR1_AWDCH_2| ADC_CFGR1_AWDCH_1| ADC_CFGR1_AWDCH_0)
	#if defined (STM32L053xx) || defined (STM32L063xx) || defined (STM32L073xx) || defined (STM32L083xx)
		#define ADC_CHANNEL_16          ((uint32_t)(ADC_CHSELR_CHSEL16)| ADC_CFGR1_AWDCH_4)
	#endif
	#define ADC_CHANNEL_17          ((uint32_t)(ADC_CHSELR_CHSEL17)| ADC_CFGR1_AWDCH_4| ADC_CFGR1_AWDCH_0)
	#define ADC_CHANNEL_18          ((uint32_t)(ADC_CHSELR_CHSEL18)| ADC_CFGR1_AWDCH_4| ADC_CFGR1_AWDCH_1)

	/* Internal channels */
	#if defined (STM32L053xx) || defined (STM32L063xx) || defined (STM32L073xx) || defined (STM32L083xx)
		#define ADC_CHANNEL_VLCD         ADC_CHANNEL_16
	#endif
	#define ADC_CHANNEL_VREFINT      ADC_CHANNEL_17
	#define ADC_CHANNEL_TEMPSENSOR   ADC_CHANNEL_18

	/* Definition of ADCx clock resources */
	#define ADC1_CLK_ENABLE()               SET_BIT(RCC->APB2ENR, (RCC_APB2ENR_ADC1EN))

	#define ADC1_FORCE_RESET()              SET_BIT(RCC->APB2RSTR, (RCC_APB2RSTR_ADC1RST))
	#define ADC1_RELEASE_RESET()            CLEAR_BIT(RCC->APB2RSTR, (RCC_APB2RSTR_ADC1RST))

	#define ADC_SINGLE_ENDED                        0x00000000U   /* dummy value */

	#define HANDLE_LOCK(__HANDLE__)                                               \
								  do{                                            \
									  if((__HANDLE__)->Lock == HANDLE_LOCKED)       \
									  {                                          \
										 return STATUS_BUSY;                        \
									  }                                          \
									  else                                       \
									  {                                          \
										 (__HANDLE__)->Lock = HANDLE_LOCKED;        \
									  }                                          \
									}while (0)

	#define HANDLE_UNLOCK(__HANDLE__)                                             \
									do{                                          \
										(__HANDLE__)->Lock = HANDLE_UNLOCKED;       \
									  }while (0)

	// @defgroup ADC_flags_definition ADC flags definition
	// @{
	//
	#define ADC_FLAG_RDY           ADC_ISR_ADRDY    //!< ADC Ready flag
	#define ADC_FLAG_EOSMP         ADC_ISR_EOSMP    //!< ADC End of Sampling flag
	#define ADC_FLAG_EOC           ADC_ISR_EOC      //!< ADC End of Regular Conversion flag
	#define ADC_FLAG_EOS           ADC_ISR_EOSEQ    //!< ADC End of Regular sequence of Conversions flag
	#define ADC_FLAG_OVR           ADC_ISR_OVR      //!< ADC overrun flag
	#define ADC_FLAG_AWD           ADC_ISR_AWD      //!< ADC Analog watchdog flag
	#define ADC_FLAG_EOCAL         ADC_ISR_EOCAL    //!< ADC Enf Of Calibration flag


	#define ADC_FLAG_ALL    (ADC_FLAG_RDY | ADC_FLAG_EOSMP | ADC_FLAG_EOC | ADC_FLAG_EOS |  \
							 ADC_FLAG_OVR | ADC_FLAG_AWD   | ADC_FLAG_EOCAL)

	//
	// @brief Verification of ADC state: enabled or disabled
	// @param __HANDLE__: ADC handle
	// @retval SET (ADC enabled) or RESET (ADC disabled)
	//
	#define ADC_IS_ENABLE(__HANDLE__)                                                    \
		   (( ((((__HANDLE__)->Instance->CR) & (ADC_CR_ADEN | ADC_CR_ADDIS)) == ADC_CR_ADEN) && \
			  ((((__HANDLE__)->Instance->ISR) & ADC_FLAG_RDY) == ADC_FLAG_RDY)                  \
			) ? SET : RESET)

	//
	// @brief Simultaneously clears and sets specific bits of the handle State
	// @note: ADC_STATE_CLR_SET() macro is merely aliased to generic macro MODIFY_REG(),
	//        the first parameter is the ADC handle State, the second parameter is the
	//        bit field to clear, the third and last parameter is the bit field to set.
	// @retval None
	//
	#define ADC_STATE_CLR_SET MODIFY_REG

	//
	// @brief  ADC state machine: ADC states definition (bitfields)
	// @note   ADC state machine is managed by bitfields, state must be compared
	//         with bit by bit.
	//         For example:
	//           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_REG_BUSY)) "
	//           " if (HAL_IS_BIT_SET(HAL_ADC_GetState(hadc1), HAL_ADC_STATE_AWD1)    ) "
	//
	// States of ADC global scope
	#define ADC_STATE_RESET             0x00000000U    //!< ADC not yet initialized or disabled
	#define ADC_STATE_READY             0x00000001U    //!< ADC peripheral ready for use
	#define ADC_STATE_BUSY_INTERNAL     0x00000002U    //!< ADC is busy due to an internal process (initialization, calibration)
	#define ADC_STATE_TIMEOUT           0x00000004U    //!< TimeOut occurrence

	// States of ADC errors
	#define ADC_STATE_ERROR_INTERNAL    0x00000010U    //!< Internal error occurrence
	#define ADC_STATE_ERROR_CONFIG      0x00000020U    //!< Configuration error occurrence
	#define ADC_STATE_ERROR_DMA         0x00000040U    //!< DMA error occurrence

	// States of ADC group regular
	#define ADC_STATE_REG_BUSY          0x00000100U    //!< A conversion on ADC group regular is ongoing or can occur (either by continuous mode,
																  //	   external trigger, low power auto power-on (if feature available), multimode ADC master control (if feature available))
	#define ADC_STATE_REG_EOC           0x00000200U    //!< Conversion data available on group regular
	#define ADC_STATE_REG_OVR           0x00000400U    //!< Overrun occurrence
	#define ADC_STATE_REG_EOSMP         0x00000800U    //!< Not available on this STM32 serie: End Of Sampling flag raised

	// States of ADC group injected
	#define ADC_STATE_INJ_BUSY          0x00001000U    //!< Not available on this STM32 serie: A conversion on group injected is ongoing or can occur (either by auto-injection mode,
																  //	   external trigger, low power auto power-on (if feature available), multimode ADC master control (if feature available))
	#define ADC_STATE_INJ_EOC           0x00002000U    //!< Not available on this STM32 serie: Conversion data available on group injected
	#define ADC_STATE_INJ_JQOVF         0x00004000U    //!< Not available on this STM32 serie: Injected queue overflow occurrence

	// States of ADC analog watchdogs
	#define ADC_STATE_AWD1              0x00010000U    //!< Out-of-window occurrence of ADC analog watchdog 1
	#define ADC_STATE_AWD2              0x00020000U    //!< Not available on this STM32 serie: Out-of-window occurrence of ADC analog watchdog 2
	#define ADC_STATE_AWD3              0x00040000U    //!< Not available on this STM32 serie: Out-of-window occurrence of ADC analog watchdog 3

	// States of ADC multi-mode
	#define ADC_STATE_MULTIMODE_SLAVE   0x00100000U    //!< Not available on this STM32 serie: ADC in multimode slave state, controlled by another ADC master (when feature available)

	// Fixed timeout values for ADC calibration, enable settling time, disable
	// settling time.
	// Values defined to be higher than worst cases: low clock frequency,
	// maximum prescaler.
	// Unit: ms
	#define ADC_CALIBRATION_TIMEOUT      10U

	// @defgroup ADC_ClockPrescaler ADC Clock Prescaler
	// @{
	//
	#define ADC_CLOCK_ASYNC_DIV1              0x00000000U                               			//!< ADC Asynchronous clock mode divided by 1
	#define ADC_CLOCK_ASYNC_DIV2              (ADC_CCR_PRESC_0)                                     //!< ADC Asynchronous clock mode divided by 2
	#define ADC_CLOCK_ASYNC_DIV4              (ADC_CCR_PRESC_1)                                     //!< ADC Asynchronous clock mode divided by 2
	#define ADC_CLOCK_ASYNC_DIV6              (ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0)                   //!< ADC Asynchronous clock mode divided by 2
	#define ADC_CLOCK_ASYNC_DIV8              (ADC_CCR_PRESC_2)                                     //!< ADC Asynchronous clock mode divided by 2
	#define ADC_CLOCK_ASYNC_DIV10             (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_0)                   //!< ADC Asynchronous clock mode divided by 2
	#define ADC_CLOCK_ASYNC_DIV12             (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1)                   //!< ADC Asynchronous clock mode divided by 2
	#define ADC_CLOCK_ASYNC_DIV16             (ADC_CCR_PRESC_2 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0) //!< ADC Asynchronous clock mode divided by 2
	#define ADC_CLOCK_ASYNC_DIV32             (ADC_CCR_PRESC_3)                                     //!< ADC Asynchronous clock mode divided by 2
	#define ADC_CLOCK_ASYNC_DIV64             (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_0)                   //!< ADC Asynchronous clock mode divided by 2
	#define ADC_CLOCK_ASYNC_DIV128            (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1)                   //!< ADC Asynchronous clock mode divided by 2
	#define ADC_CLOCK_ASYNC_DIV256            (ADC_CCR_PRESC_3 | ADC_CCR_PRESC_1 | ADC_CCR_PRESC_0) //!< ADC Asynchronous clock mode divided by 2

	#define ADC_CLOCK_SYNC_PCLK_DIV1         ((uint32_t)ADC_CFGR2_CKMODE)    //!< Synchronous clock mode divided by 1
																			 //    This configuration must be enabled only if PCLK has a 50%
																			 //    duty clock cycle (APB prescaler configured inside the RCC must be bypassed and the system clock
																			 //    must by 50% duty cycle)
	#define ADC_CLOCK_SYNC_PCLK_DIV2         ((uint32_t)ADC_CFGR2_CKMODE_0)  //!< Synchronous clock mode divided by 2
	#define ADC_CLOCK_SYNC_PCLK_DIV4         ((uint32_t)ADC_CFGR2_CKMODE_1)  //!< Synchronous clock mode divided by 4

	// @defgroup ADC_Resolution ADC Resolution
	// @{
	//
	#define ADC_RESOLUTION_12B      0x00000000U          			 //!< ADC 12-bit resolution
	#define ADC_RESOLUTION_10B      ((uint32_t)ADC_CFGR1_RES_0)      //!< ADC 10-bit resolution
	#define ADC_RESOLUTION_8B       ((uint32_t)ADC_CFGR1_RES_1)      //!< ADC 8-bit resolution
	#define ADC_RESOLUTION_6B       ((uint32_t)ADC_CFGR1_RES)        //!< ADC 6-bit resolution

	// @defgroup ADC_Data_align ADC conversion data alignment
	// @{
	//
	#define ADC_DATAALIGN_RIGHT      0x00000000U
	#define ADC_DATAALIGN_LEFT       ((uint32_t)ADC_CFGR1_ALIGN)

	// @defgroup ADC_Scan_mode ADC Scan mode
	// @{
	//
	// Note: Scan mode values must be compatible with other STM32 devices having
	//       a configurable sequencer.
	//       Scan direction setting values are defined by taking in account
	//       already defined values for other STM32 devices:
	//         ADC_SCAN_DISABLE         ((uint32_t)0x00000000)
	//         ADC_SCAN_ENABLE          ((uint32_t)0x00000001)
	//       Scan direction forward is considered as default setting equivalent
	//       to scan enable.
	//       Scan direction backward is considered as additional setting.
	//       In case of migration from another STM32 device, the user will be
	//       warned of change of setting choices with assert check.
	#define ADC_SCAN_DIRECTION_FORWARD        0x00000001U        //!< Scan direction forward: from channel 0 to channel 18
	#define ADC_SCAN_DIRECTION_BACKWARD       0x00000002U        //!< Scan direction backward: from channel 18 to channel 0

	#define ADC_SCAN_ENABLE         ADC_SCAN_DIRECTION_FORWARD             // For compatibility with other STM32 devices

	// @defgroup ADC_EOCSelection ADC EOC Selection
	// @{
	//
	#define ADC_EOC_SINGLE_CONV         ((uint32_t) ADC_ISR_EOC)
	#define ADC_EOC_SEQ_CONV            ((uint32_t) ADC_ISR_EOS)
	#define ADC_EOC_SINGLE_SEQ_CONV     ((uint32_t)(ADC_ISR_EOC | ADC_ISR_EOS))  //!< reserved for future use

	// @defgroup ADC_regular_external_trigger_source ADC External Trigger Source
	// @{
	//
	#define ADC_EXTERNALTRIGCONV_T6_TRGO            0x00000000U
	#define ADC_EXTERNALTRIGCONV_T21_CC2            (ADC_CFGR1_EXTSEL_0)
	#define ADC_EXTERNALTRIGCONV_T2_TRGO            (ADC_CFGR1_EXTSEL_1)
	#define ADC_EXTERNALTRIGCONV_T2_CC4             (ADC_CFGR1_EXTSEL_1 | ADC_CFGR1_EXTSEL_0)
	#define ADC_EXTERNALTRIGCONV_T22_TRGO           (ADC_CFGR1_EXTSEL_2)
	#define ADC_EXTERNALTRIGCONV_T3_TRGO            (ADC_CFGR1_EXTSEL_2 | ADC_CFGR1_EXTSEL_1)
	#define ADC_EXTERNALTRIGCONV_EXT_IT11           (ADC_CFGR1_EXTSEL_2 | ADC_CFGR1_EXTSEL_1 | ADC_CFGR1_EXTSEL_0)
	#define ADC_SOFTWARE_START                      (ADC_CFGR1_EXTSEL + 1U)

	// @defgroup ADC_regular_external_trigger_edge ADC External Trigger Source Edge for Regular Group
	// @{
	//
	#define ADC_EXTERNALTRIGCONVEDGE_NONE           0x00000000U
	#define ADC_EXTERNALTRIGCONVEDGE_RISING         ((uint32_t)ADC_CFGR1_EXTEN_0)
	#define ADC_EXTERNALTRIGCONVEDGE_FALLING        ((uint32_t)ADC_CFGR1_EXTEN_1)
	#define ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING  ((uint32_t)ADC_CFGR1_EXTEN)

	// @defgroup ADC_Overrun ADC Overrun
	// @{
	//
	#define ADC_OVR_DATA_PRESERVED              0x00000000U
	#define ADC_OVR_DATA_OVERWRITTEN            ((uint32_t)ADC_CFGR1_OVRMOD)

	// @defgroup ADC_sampling_times ADC Sampling Cycles
	// @{
	//
	#define ADC_SAMPLETIME_1CYCLE_5       0x00000000U   			                      //!<  ADC sampling time 1.5 cycle
	#define ADC_SAMPLETIME_3CYCLES_5      ((uint32_t)ADC_SMPR_SMPR_0)                     //!<  ADC sampling time 3.5 CYCLES
	#define ADC_SAMPLETIME_7CYCLES_5      ((uint32_t)ADC_SMPR_SMPR_1)                     //!<  ADC sampling time 7.5 CYCLES
	#define ADC_SAMPLETIME_12CYCLES_5     ((uint32_t)(ADC_SMPR_SMPR_1 | ADC_SMPR_SMPR_0)) //!<  ADC sampling time 12.5 CYCLES
	#define ADC_SAMPLETIME_19CYCLES_5     ((uint32_t)ADC_SMPR_SMPR_2)                     //!<  ADC sampling time 19.5 CYCLES
	#define ADC_SAMPLETIME_39CYCLES_5     ((uint32_t)(ADC_SMPR_SMPR_2 | ADC_SMPR_SMPR_0)) //!<  ADC sampling time 39.5 CYCLES
	#define ADC_SAMPLETIME_79CYCLES_5     ((uint32_t)(ADC_SMPR_SMPR_2 | ADC_SMPR_SMPR_1)) //!<  ADC sampling time 79.5 CYCLES
	#define ADC_SAMPLETIME_160CYCLES_5    ((uint32_t)ADC_SMPR_SMPR)                       //!<  ADC sampling time 160.5 CYCLES

	//
	// @brief Check if no conversion on going on regular group
	// @param __HANDLE__: ADC handle
	// @retval SET (conversion is on going) or RESET (no conversion is on going)
	//
	#define ADC_IS_CONVERSION_ONGOING_REGULAR(__HANDLE__)                          \
	    (( (((__HANDLE__)->Instance->CR) & ADC_CR_ADSTART) == RESET                  \
	    ) ? RESET : SET)

	// Fixed timeout values for ADC calibration, enable settling time, disable
	// settling time.
	// Values defined to be higher than worst cases: low clocks freq,
	// maximum prescalers.
	// Unit: ms
	#define ADC_ENABLE_TIMEOUT            10U
	#define ADC_DISABLE_TIMEOUT           10U
	#define ADC_STOP_CONVERSION_TIMEOUT   10U

	// @defgroup ADC_Error_Code ADC Error Code
	// @{
	//
	#define ADC_ERROR_NONE        0x00U   //!< No error
	#define ADC_ERROR_INTERNAL    0x01U   //!< ADC IP internal error (problem of clocking,
										  //				  enable/disable, erroneous state, ...)
	#define ADC_ERROR_OVR         0x02U   //!< Overrun error
	#define ADC_ERROR_DMA         0x04U   //!< DMA transfer error

	//
	// @brief Verification of hardware constraints before ADC can be disabled
	// @param __HANDLE__: ADC handle
	// @retval SET (ADC can be disabled) or RESET (ADC cannot be disabled)
	//
	#define ADC_DISABLING_CONDITIONS(__HANDLE__)                             \
		   (( ( ((__HANDLE__)->Instance->CR) &                                     \
				(ADC_CR_ADSTART | ADC_CR_ADEN)) == ADC_CR_ADEN   \
			) ? SET : RESET)

	//
	// @brief Clear the ADC's pending flags
	// @param __HANDLE__: ADC handle.
	// @param __FLAG__: ADC flag.
	// @retval None
	//
	// Note: bit cleared bit by writing 1
	#define ADC_CLEAR_FLAG(__HANDLE__, __FLAG__) \
		(((__HANDLE__)->Instance->ISR) = (__FLAG__))

	//
	// @brief Disable the ADC peripheral
	// @param __HANDLE__: ADC handle
	// @retval None
	//
	#define ADC_DISABLE(__HANDLE__)                                          \
		do{                                                                          \
			(__HANDLE__)->Instance->CR |= ADC_CR_ADDIS;                           \
			ADC_CLEAR_FLAG((__HANDLE__), (ADC_FLAG_EOSMP | ADC_FLAG_RDY)); \
		} while(0)

	//
	// @brief Disable the ADC end of conversion interrupt.
	// @param __HANDLE__: ADC handle.
	// @param __INTERRUPT__: ADC interrupt.
	// @retval None
	//
	#define ADC_DISABLE_IT(__HANDLE__, __INTERRUPT__) \
		(((__HANDLE__)->Instance->IER) &= ~(__INTERRUPT__))

	// @defgroup ADC_interrupts_definition ADC Interrupts Definition
	// @{
	//
	#define ADC_IT_RDY           ADC_IER_ADRDYIE     //!< ADC Ready (ADRDY) interrupt source
	#define ADC_IT_EOSMP         ADC_IER_EOSMPIE     //!< ADC End of Sampling interrupt source
	#define ADC_IT_EOC           ADC_IER_EOCIE       //!< ADC End of Regular Conversion interrupt source
	#define ADC_IT_EOS           ADC_IER_EOSEQIE     //!< ADC End of Regular sequence of Conversions interrupt source
	#define ADC_IT_OVR           ADC_IER_OVRIE       //!< ADC overrun interrupt source
	#define ADC_IT_AWD           ADC_IER_AWDIE       //!< ADC Analog watchdog 1 interrupt source
	#define ADC_IT_EOCAL         ADC_IER_EOCALIE     //!< ADC End of Calibration interrupt source

	//
	// @brief Clear ADC error code (set it to error code: "no error")
	// @param __HANDLE__: ADC handle
	// @retval None
	//
	#define ADC_CLEAR_ERRORCODE(__HANDLE__)                                        \
		((__HANDLE__)->ErrorCode = ADC_ERROR_NONE)

	//
	// @brief Configuration of ADC clock & prescaler: clock source PCLK or Asynchronous with selectable prescaler
	// @param __HANDLE__: ADC handle
	// @retval None
	//
	#define ADC_CLOCK_PRESCALER(__HANDLE__)                                       \
		do{                                                                               \
			if ((((__HANDLE__)->Init.ClockPrescaler) == ADC_CLOCK_SYNC_PCLK_DIV1) ||  \
				(((__HANDLE__)->Init.ClockPrescaler) == ADC_CLOCK_SYNC_PCLK_DIV2) ||  \
				(((__HANDLE__)->Init.ClockPrescaler) == ADC_CLOCK_SYNC_PCLK_DIV4))    \
			{                                                                             \
				(__HANDLE__)->Instance->CFGR2 &= ~(ADC_CFGR2_CKMODE);                       \
				(__HANDLE__)->Instance->CFGR2 |=  (__HANDLE__)->Init.ClockPrescaler;        \
			}                                                                             \
			else                                                                          \
			{                                                                             \
				/* CKMOD bits must be reset */                                              \
				(__HANDLE__)->Instance->CFGR2 &= ~(ADC_CFGR2_CKMODE);                       \
				ADC->CCR &= ~(ADC_CCR_PRESC);                                               \
				ADC->CCR |=  (__HANDLE__)->Init.ClockPrescaler;                             \
			}                                                                             \
		} while(0)

	//
	// @brief Enable the ADC Low Frequency mode.
	// @param _LOW_FREQUENCY_MODE_: Low Frequency mode.
	// @retval None
	//
	#define ADC_CCR_LOWFREQUENCY(_LOW_FREQUENCY_MODE_) ((_LOW_FREQUENCY_MODE_) << 25U)

	//
	// @brief Enable ADC scan mode to convert multiple ranks with sequencer.
	// @param _SCAN_MODE_: Scan conversion mode.
	// @retval None
	//
	#define ADC_SCANDIR(_SCAN_MODE_)                                   \
		( ( (_SCAN_MODE_) == (ADC_SCAN_DIRECTION_BACKWARD)                           \
			)? (ADC_CFGR1_SCANDIR) : (0x00000000U)                                      \
		)

	//
	// @brief Enable ADC continuous conversion mode.
	// @param _CONTINUOUS_MODE_: Continuous mode.
	// @retval None
	//
	#define ADC_CONTINUOUS(_CONTINUOUS_MODE_) ((_CONTINUOUS_MODE_) << 13U)

	//
	// @brief Enable the ADC DMA continuous request.
	// @param _DMAContReq_MODE_: DMA continuous request mode.
	// @retval None
	//
	#define ADC_DMACONTREQ(_DMAContReq_MODE_) ((_DMAContReq_MODE_) << 1U)

	//
	// @brief Enable the ADC Auto Delay.
	// @param _AutoDelay_: Auto delay bit enable or disable.
	// @retval None
	//
	#define ADC_CFGR1_AutoDelay(_AutoDelay_) ((_AutoDelay_) << 14U)

	//
	// @brief Enable the ADC LowPowerAutoPowerOff.
	// @param _AUTOFF_: AutoOff bit enable or disable.
	// @retval None
	//
	#define ADC_CFGR1_AUTO_OFF(_AUTOFF_) ((_AUTOFF_) << 15U)

	// @defgroup ADC_rank ADC rank
	// @{
	//
	#define ADC_RANK_CHANNEL_NUMBER                 0x00001000U  //!< Enable the rank of the selected channels. Number of ranks in the sequence is defined by number of channels enabled, rank of each channel is defined by channel number (channel 0 fixed on rank 0, channel 1 fixed on rank1, ...)
	#define ADC_RANK_NONE                           0x00001001U  //!< Disable the selected rank (selected channel) from sequencer

	// @defgroup ADC_Channel_AWD_Masks ADC Channel Masks
	// @{
	//
	#define ADC_CHANNEL_MASK        0x0007FFFFU
	#define ADC_CHANNEL_AWD_MASK    0x7C000000U

	// Delay for temperature sensor stabilization time.
	// Maximum delay is 10us (refer to device datasheet, parameter tSTART).
	// Unit: us
	#define ADC_TEMPSENSOR_DELAY_US ((uint32_t) 10U)

	//
	// @brief Verification of hardware constraints before ADC can be enabled
	// @param __HANDLE__: ADC handle
	// @retval SET (ADC can be enabled) or RESET (ADC cannot be enabled)
	//
	#define ADC_ENABLING_CONDITIONS(__HANDLE__)           \
		(( ( ((__HANDLE__)->Instance->CR) &                  \
			(ADC_CR_ADCAL | ADC_CR_ADSTP | ADC_CR_ADSTART | \
				ADC_CR_ADDIS | ADC_CR_ADEN )                   \
			) == RESET                                       \
		) ? SET : RESET)

	//
	// @brief Enable the ADC peripheral
	// @param __HANDLE__: ADC handle
	// @retval None
	//
	#define ADC_ENABLE(__HANDLE__) ((__HANDLE__)->Instance->CR |= ADC_CR_ADEN)

	//
	// @brief Get the selected ADC's flag status.
	// @param __HANDLE__: ADC handle.
	// @param __FLAG__: ADC flag.
	// @retval None
	//
	#define ADC_GET_FLAG(__HANDLE__, __FLAG__) \
	    ((((__HANDLE__)->Instance->ISR) & (__FLAG__)) == (__FLAG__))

	// Delay for ADC stabilization time.
	// Maximum delay is 1us (refer to device datasheet, parameter tSTART).
	// Unit: us
	#define ADC_STAB_DELAY_US       1U

	//
	// @brief Test if conversion trigger of regular group is software start
	//        or external trigger.
	// @param __HANDLE__: ADC handle
	// @retval SET (software start) or RESET (external trigger)
	//
	#define ADC_IS_SOFTWARE_START_REGULAR(__HANDLE__)                              \
		(((__HANDLE__)->Instance->CFGR1 & ADC_CFGR1_EXTEN) == RESET)

	//
	// @brief Enable the ADC end of conversion interrupt.
	// @param __HANDLE__: ADC handle.
	// @param __INTERRUPT__: ADC Interrupt.
	// @retval None
	//
	#define ADC_ENABLE_IT(__HANDLE__, __INTERRUPT__)  \
		(((__HANDLE__)->Instance->IER) |= (__INTERRUPT__))

	//
	// @brief  Enable the specified DMA Channel.
	// @param  __HANDLE__: DMA handle
	// @retval None.
	//
	#define DMA_ENABLE(__HANDLE__)        ((__HANDLE__)->Instance->CCR |=  DMA_CCR_EN)

	//
	// @brief  Disable the specified DMA Channel.
	// @param  __HANDLE__: DMA handle
	// @retval None.
	//
	#define DMA_DISABLE(__HANDLE__)       ((__HANDLE__)->Instance->CCR &=  ~DMA_CCR_EN)

	// @defgroup DMA_Data_transfer_direction DMA Data Transfer directions
	// @{
	//
	#define DMA_PERIPH_TO_MEMORY         0x00000000U 			       //!< Peripheral to memory direction
	#define DMA_MEMORY_TO_PERIPH         ((uint32_t)DMA_CCR_DIR)       //!< Memory to peripheral direction
	#define DMA_MEMORY_TO_MEMORY         ((uint32_t)(DMA_CCR_MEM2MEM)) //!< Memory to memory direction

	#define IS_DMA_DIRECTION(DIRECTION) (((DIRECTION) == DMA_PERIPH_TO_MEMORY ) || \
		((DIRECTION) == DMA_MEMORY_TO_PERIPH)  || \
		((DIRECTION) == DMA_MEMORY_TO_MEMORY))

	//
	// @brief  Enables the specified DMA Channel interrupts.
	// @param  __HANDLE__: DMA handle
	// @param __INTERRUPT__: specifies the DMA interrupt sources to be enabled or disabled.
	//          This parameter can be any combination of the following values:
	//            @arg DMA_IT_TC:  Transfer complete interrupt mask
	//            @arg DMA_IT_HT:  Half transfer complete interrupt mask
	//            @arg DMA_IT_TE:  Transfer error interrupt mask
	// @retval None
	//
	#define DMA_ENABLE_IT(__HANDLE__, __INTERRUPT__)   ((__HANDLE__)->Instance->CCR |= (__INTERRUPT__))

	//
	// @brief  Disables the specified DMA Channel interrupts.
	// @param  __HANDLE__: DMA handle
	// @param __INTERRUPT__: specifies the DMA interrupt sources to be enabled or disabled.
	//          This parameter can be any combination of the following values:
	//            @arg DMA_IT_TC:  Transfer complete interrupt mask
	//            @arg DMA_IT_HT:  Half transfer complete interrupt mask
	//            @arg DMA_IT_TE:  Transfer error interrupt mask
	// @retval None
	//
	#define DMA_DISABLE_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->Instance->CCR &= ~(__INTERRUPT__))

	// @defgroup DMA_interrupt_enable_definitions DMA Interrupt Definitions
	// @{
	//
	#define DMA_IT_TC                         ((uint32_t)DMA_CCR_TCIE)
	#define DMA_IT_HT                         ((uint32_t)DMA_CCR_HTIE)
	#define DMA_IT_TE                         ((uint32_t)DMA_CCR_TEIE)

	const short temptable_998[][2] = {
		{    1 * OVERSAMPLENR, DUMMY_THERMISTOR_998_VALUE },
		{ 1023 * OVERSAMPLENR, DUMMY_THERMISTOR_998_VALUE }
	};

	//
	// Variable initialization
	//

	static ADC_HandleTypeDef AdcHandle;

	Timers::TIM_HandleTypeDef htim2;

	/* Variable containing ADC conversions results */
	static __IO uint16_t   aADCxConvertedValues[ADCCONVERTEDVALUES_BUFFER_SIZE];

	/* Variable to report ADC sequencer status */
	static uint8_t         ubSequenceCompleted = RESET;     /* Set when all ranks of the sequence have been converted */

	// Init min and max temp with extreme values to prevent false errors during startup
	static int16_t minttemp_raw = PRINTHEAD_RAW_LO_TEMP;
	static int16_t maxttemp_raw = PRINTHEAD_RAW_HI_TEMP;
	static int16_t minttemp = 0;
	static int16_t maxttemp = 16383;

	// Init min and max volt with extreme values to prevent false errors during startup
	static int16_t mintvolt_raw = BATTERY_RAW_LO_VOLT;
	static int16_t maxtvolt_raw = BATTERY_RAW_HI_VOLT;
	static int16_t mintvolt = 0;
//	static int16_t maxtvolt = 16383;

	static void* heater_ttbl_map = (void*)PRINTHEAD_TEMPTABLE;
	static uint8_t heater_ttbllen_map = PRINTHEAD_TEMPTABLE_LEN;

	static void* battery_ttbl_map = (void*)BATTERY_VOLTTABLE;
	static uint8_t battery_ttbllen_map = BATTERY_VOLTTABLE_LEN;

	static volatile bool sens_meas_ready = false;

	static uint16_t raw_temp_value = 0U;
	static uint16_t raw_volt_value = 0U;

	static int16_t current_temperature_raw = 0;
	static int16_t current_voltage_raw = 0;

	volatile bool in_temp_isr = false;

	//
	// Private function prototypes
	//

	//
	// @brief  Perform an ADC automatic self-calibration
	//         Calibration prerequisite: ADC must be disabled (execute this
	//         function before HAL_ADC_Start() or after HAL_ADC_Stop() ).
	// @note   Calibration factor can be read after calibration, using function
	//         HAL_ADC_GetValue() (value on 7 bits: from DR[6;0]).
	// @param  hadc       ADC handle
	// @param  SingleDiff: Selection of single-ended or differential input
	//          This parameter can be only of the following values:
	//            @arg ADC_SINGLE_ENDED: Channel in mode input single ended
	// @retval HAL status
	//
	StatusTypeDef ADCEx_Calibration_Start(ADC_HandleTypeDef* hadc, uint32_t SingleDiff);

	//
	// @brief  Stop ADC conversion.
	// @note   Prerequisite condition to use this function: ADC conversions must be
	//         stopped to disable the ADC.
	// @param  hadc: ADC handle
	// @retval status.
	//
	static StatusTypeDef ADC_ConversionStop(ADC_HandleTypeDef* hadc);

	//
	// @brief  Disable the selected ADC.
	// @note   Prerequisite condition to use this function: ADC conversions must be
	//         stopped.
	// @param  hadc: ADC handle
	// @retval status.
	///
	static StatusTypeDef ADC_Disable(ADC_HandleTypeDef* hadc);

	//
	// @brief ADC MSP de-initialization
	//        This function frees the hardware resources used in this example:
	//          - Disable clock of ADC peripheral
	//          - Revert GPIO associated to the peripheral channels to their default state
	//          - Revert NVIC associated to the peripheral interruptions to its default state
	// @param hadc: ADC handle pointer
	// @retval None
	//
	void ADC_MspDeInit(ADC_HandleTypeDef *hadc);

	//
	// @brief  Deinitialize the ADC peripheral registers to their default reset
	//         values, with deinitialization of the ADC MSP.
	// @note   For devices with several ADCs: reset of ADC common registers is done
	//         only if all ADCs sharing the same common group are disabled.
	//         If this is not the case, reset of these common parameters reset is
	//         bypassed without error reporting: it can be the intended behavior in
	//         case of reset of a single ADC while the other ADCs sharing the same
	//         common group is still running.
	// @param  hadc: ADC handle
	// @retval HAL status
	//
	StatusTypeDef ADC_DeInit(ADC_HandleTypeDef* hadc);

	void ADC1_CHANNEL_VOLTAGE_GPIO_CLK_ENABLE();
	void ADC1_CHANNEL_TEMPERATURE_GPIO_CLK_ENABLE();

	//
	// @brief ADC MSP initialization
	//        This function configures the hardware resources used in this example:
	//          - Enable clock of ADC peripheral
	//          - Configure the GPIO associated to the peripheral channels
	//          - Configure the NVIC associated to the peripheral interruptions
	// @param hadc: ADC handle pointer
	// @retval None
	//
	void ADC_MspInit(ADC_HandleTypeDef *hadc);

	//
	// @brief  Initialize the ADC peripheral and regular group according to
	//         parameters specified in structure "ADC_InitTypeDef".
	// @note   As prerequisite, ADC clock must be configured at RCC top level
	//         depending on possible clock sources: APB clock of HSI clock.
	//         See commented example code below that can be copied and uncommented
	//         into HAL_ADC_MspInit().
	// @note   Possibility to update parameters on the fly:
	//         This function initializes the ADC MSP (HAL_ADC_MspInit()) only when
	//         coming from ADC state reset. Following calls to this function can
	//         be used to reconfigure some parameters of ADC_InitTypeDef
	//         structure on the fly, without modifying MSP configuration. If ADC
	//         MSP has to be modified again, HAL_ADC_DeInit() must be called
	//         before HAL_ADC_Init().
	//         The setting of these parameters is conditioned to ADC state.
	//         For parameters constraints, see comments of structure
	//         "ADC_InitTypeDef".
	// @note   This function configures the ADC within 2 scopes: scope of entire
	//         ADC and scope of regular group. For parameters details, see comments
	//         of structure "ADC_InitTypeDef".
	// @note   When device is in mode low-power (low-power run, low-power sleep or stop mode),
	//         function "HAL_ADCEx_EnableVREFINT()" must be called before function HAL_ADC_Init()
	//         (in case of previous ADC operations: function HAL_ADC_DeInit() must be called first).
	//         In case of internal temperature sensor to be measured:
	//         function "HAL_ADCEx_EnableVREFINTTempSensor()" must be called similarilly.
	// @param  hadc: ADC handle
	// @retval status
	//
	StatusTypeDef ADC_Init(ADC_HandleTypeDef* hadc);

	//
	// @brief  Delay micro seconds
	// @param  microSecond : delay
	// @retval None
	//
	void ADC_DelayMicroSecond(uint32_t microSecond);

	//
	// @brief  Configure a channel to be assigned to ADC group regular.
	// @note   In case of usage of internal measurement channels:
	//         VrefInt/Vlcd(STM32L0x3xx only)/TempSensor.
	//         Sampling time constraints must be respected (sampling time can be
	//         adjusted in function of ADC clock frequency and sampling time
	//         setting).
	//         Refer to device datasheet for timings values, parameters TS_vrefint,
	//         TS_vlcd (STM32L0x3xx only), TS_temp (values rough order: 5us to 17us).
	//         These internal paths can be be disabled using function
	//         HAL_ADC_DeInit().
	// @note   Possibility to update parameters on the fly:
	//         This function initializes channel into ADC group regular,
	//         following calls to this function can be used to reconfigure
	//         some parameters of structure "ADC_ChannelConfTypeDef" on the fly,
	//         without resetting the ADC.
	//         The setting of these parameters is conditioned to ADC state:
	//         Refer to comments of structure "ADC_ChannelConfTypeDef".
	// @param  hadc: ADC handle
	// @param  sConfig: Structure of ADC channel assigned to ADC group regular.
	// @retval status
	//
	StatusTypeDef ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);

	//
	// @brief  Enable the selected ADC.
	// @note   Prerequisite condition to use this function: ADC must be disabled
	//         and voltage regulator must be enabled (done into HAL_ADC_Init()).
	// @note   If low power mode AutoPowerOff is enabled, power-on/off phases are
	//         performed automatically by hardware.
	//         In this mode, this function is useless and must not be called because
	//         flag ADC_FLAG_RDY is not usable.
	//         Therefore, this function must be called under condition of
	//         "if (hadc->Init.LowPowerAutoPowerOff != ENABLE)".
	// @param  hadc: ADC handle
	// @retval status.
	//
	StatusTypeDef ADC_Enable(ADC_HandleTypeDef* hadc);

	//
	// @brief  Conversion complete callback in non blocking mode
	// @param  AdcHandle : ADC handle
	// @note   This example shows a simple way to report end of conversion
	//         and get conversion result. You can add your own implementation.
	// @retval None
	//
	void ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle);

	//
	// @brief  Conversion DMA half-transfer callback in non blocking mode
	// @param  hadc: ADC handle
	// @retval None
	//
	void ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);

	//
	// @brief  ADC error callback in non blocking mode
	//        (ADC conversion with interruption or transfer by DMA)
	// @param  hadc: ADC handle
	// @retval None
	//
	void ADC_ErrorCallback(ADC_HandleTypeDef *hadc);

	//
	// @brief  DMA transfer complete callback.
	// @param  hdma: pointer to DMA handle.
	// @retval None
	//
	static void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma);

	//
	// @brief  DMA half transfer complete callback.
	// @param  hdma: pointer to DMA handle.
	// @retval None
	//
	static void ADC_DMAHalfConvCplt(DMA_HandleTypeDef *hdma);

	//
	// @brief  DMA error callback.
	// @param  hdma: pointer to DMA handle.
	// @retval None
	//
	static void ADC_DMAError(DMA_HandleTypeDef *hdma);

	//
	// @brief  Sets the DMA Transfer parameter.
	// @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
	//                     the configuration information for the specified DMA Channel.
	// @param  SrcAddress: The source memory Buffer address
	// @param  DstAddress: The destination memory Buffer address
	// @param  DataLength: The length of data to be transferred from source to destination
	// @retval HAL status
	//
	static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);

	//
	// @brief  Start the DMA Transfer with interrupt enabled.
	// @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
	//                     the configuration information for the specified DMA Channel.
	// @param  SrcAddress: The source memory Buffer address
	// @param  DstAddress: The destination memory Buffer address
	// @param  DataLength: The length of data to be transferred from source to destination
	// @retval status
	//
	StatusTypeDef DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);

	//
	// @brief  Enable ADC, start conversion of regular group and transfer result through DMA.
	// @note   Interruptions enabled in this function:
	//         overrun (if applicable), DMA half transfer, DMA transfer complete.
	//         Each of these interruptions has its dedicated callback function.
	// @param  hadc: ADC handle
	// @param  pData: Destination Buffer address.
	// @param  Length: Length of data to be transferred from ADC peripheral to memory (in bytes)
	// @retval status.
	//
	StatusTypeDef ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);

	//
	// @brief  ADC configuration
	// @param  None
	// @retval None
	//
	void ADC_Config(void);

	//
	// @brief  Enable ADC, start conversion of regular group.
	// @note   Interruptions enabled in this function: None.
	// @param  hadc: ADC handle
	// @retval status
	///
	StatusTypeDef ADC_Start(ADC_HandleTypeDef* hadc);

	//
	// Called from the Temperature ISR
	//
	void isr();

	// Derived from RepRap FiveD extruder::getTemperature()
	// For hot end temperature measurement.
	float32_t analog2temp(int32_t raw);

	// For battery voltage measurement.
	float32_t analog2volt(int32_t raw);

	//
	// Get raw temperatures
	//
	void set_current_sens_raw();

	//
	// Temperature Error Handlers
	//
	void _sens_error(const char * const serial_msg);

	void max_sens_error();
	void min_sens_error();

	//
	// Namespace body
	//

	//
	// @brief  Perform an ADC automatic self-calibration
	//         Calibration prerequisite: ADC must be disabled (execute this
	//         function before HAL_ADC_Start() or after HAL_ADC_Stop() ).
	// @note   Calibration factor can be read after calibration, using function
	//         HAL_ADC_GetValue() (value on 7 bits: from DR[6;0]).
	// @param  hadc       ADC handle
	// @param  SingleDiff: Selection of single-ended or differential input
	//          This parameter can be only of the following values:
	//            @arg ADC_SINGLE_ENDED: Channel in mode input single ended
	// @retval HAL status
	//
	StatusTypeDef ADCEx_Calibration_Start(ADC_HandleTypeDef* hadc, uint32_t SingleDiff)
	{
		StatusTypeDef tmp_hal_status = STATUS_OK;
		uint32_t tickstart = 0U;
		uint32_t backup_setting_adc_dma_transfer = 0U; // Note: Variable not declared as volatile because register read is already declared as volatile

		// Process locked
		HANDLE_LOCK(hadc);

		// Calibration prerequisite: ADC must be disabled.
		if (ADC_IS_ENABLE(hadc) == RESET)
		{
			// Set ADC state
			ADC_STATE_CLR_SET(hadc->State,
				ADC_STATE_REG_BUSY,
				ADC_STATE_BUSY_INTERNAL);

			// Disable ADC DMA transfer request during calibration
			// Note: Specificity of this STM32 serie: Calibration factor is
			//       available in data register and also transfered by DMA.
			//       To not insert ADC calibration factor among ADC conversion data
			//       in array variable, DMA transfer must be disabled during
			//       calibration.
			backup_setting_adc_dma_transfer = READ_BIT(hadc->Instance->CFGR1, ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG);
			CLEAR_BIT(hadc->Instance->CFGR1, ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG);

			// Start ADC calibration
			hadc->Instance->CR |= ADC_CR_ADCAL;

			tickstart = Timers::GetTick();

			// Wait for calibration completion
			while(isBitSet(hadc->Instance->CR, ADC_CR_ADCAL))
			{
				if((Timers::GetTick() - tickstart) > ADC_CALIBRATION_TIMEOUT)
				{
					// Update ADC state machine to error
					ADC_STATE_CLR_SET(hadc->State,
						ADC_STATE_BUSY_INTERNAL,
						ADC_STATE_ERROR_INTERNAL);

					// Process unlocked
					HANDLE_UNLOCK(hadc);

					return STATUS_ERROR;
				}
			}

			// Restore ADC DMA transfer request after calibration
			SET_BIT(hadc->Instance->CFGR1, backup_setting_adc_dma_transfer);

			// Set ADC state
			ADC_STATE_CLR_SET(hadc->State,
				ADC_STATE_BUSY_INTERNAL,
				ADC_STATE_READY);
		}
		else
		{
			// Update ADC state machine to error
			SET_BIT(hadc->State, ADC_STATE_ERROR_CONFIG);

			tmp_hal_status = STATUS_ERROR;
		}

		// Process unlocked
		HANDLE_UNLOCK(hadc);

		// Return function status
		return tmp_hal_status;
	}

	//
	// Initialize the temperature manager
	// The manager is implemented by periodic calls to manage_heater()
	///
	void init() {

		ADC_Config();

		// Run the ADC calibration in single-ended mode
		if (ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) != STATUS_OK)
		{
			// Calibration Error
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}




		// Set the timer pre-scaler
		// Generally we use a divider of 8, resulting in a 2MHz timer
		// frequency on a 16MHz MCU. If you are going to change this, be
		// sure to regenerate speed_lookuptable.h with
		// create_speed_lookuptable.py
		Timers::TIM_Base_InitTypeDef tim2_Init;
		tim2_Init.Prescaler = 64;
		tim2_Init.CounterMode = TIM_COUNTERMODE_UP;
		// Init Stepper ISR to 4 us Hz
		tim2_Init.Period = 256;
		tim2_Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

		htim2.Instance = TIM2;
		htim2.Init = tim2_Init;
		htim2.Channel = Timers::TIM_ACTIVE_CHANNEL_CLEARED;

		if (STATUS_ERROR == TIM_Base_Init(&htim2)) {
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		// Set the timer pre-scaler
		// Generally we use a divider of 64, resulting in a 250KHz timer
		// frequency on a 16MHz MCU.
		//TIM2->PSC = 64; // 1/64 prescaler

		// Init Stepper ISR to 4 us
		//TIM2->ARR = 256;

		// Enable update interrupts
		//TIM2->DIER |= TIM_DIER_UIE;
		TIM_ENABLE_IT(&htim2, TIM_DIER_UIE);

		// zero counter for initial start
		//TIM2->CNT = 0;
		TIM_RESET_COUNTER(&htim2);

		// Use timer TIM2 for temperature measurement
		TIM_SET_AUTORELOAD(&htim2, 128);
		NVIC_EnableIRQ(TIM2_IRQn);

		// Wait for temperature measurement to settle
		Timers::Delay(250);

		minttemp = PRINTHEAD_MINTEMP;
		while (analog2temp(minttemp_raw) < PRINTHEAD_MINTEMP) {
			if (PRINTHEAD_RAW_LO_TEMP < PRINTHEAD_RAW_HI_TEMP) {
				minttemp_raw += OVERSAMPLENR;
			} else {
				minttemp_raw -= OVERSAMPLENR;
			}
		}
		maxttemp = PRINTHEAD_MAXTEMP;
		while (analog2temp(maxttemp_raw) > PRINTHEAD_MAXTEMP) {
			if (PRINTHEAD_RAW_LO_TEMP < PRINTHEAD_RAW_HI_TEMP) {
				maxttemp_raw -= OVERSAMPLENR;
			} else {
				maxttemp_raw += OVERSAMPLENR;
			}
		}

		mintvolt = BATTERY_MINVOLT;
		while (analog2volt(mintvolt_raw) < BATTERY_MINVOLT) {
			if (BATTERY_RAW_LO_VOLT < BATTERY_RAW_HI_VOLT) {
				mintvolt_raw += OVERSAMPLENR;
			} else {
				mintvolt_raw -= OVERSAMPLENR;
			}
		}
		maxttemp = BATTERY_MAXVOLT;
		while (analog2volt(maxtvolt_raw) > BATTERY_MAXVOLT) {
			if (BATTERY_RAW_LO_VOLT < BATTERY_RAW_HI_VOLT) {
				maxtvolt_raw -= OVERSAMPLENR;
			} else {
				maxtvolt_raw += OVERSAMPLENR;
			}
		}
	}

	//
	// @brief  Stop ADC conversion.
	// @note   Prerequisite condition to use this function: ADC conversions must be
	//         stopped to disable the ADC.
	// @param  hadc: ADC handle
	// @retval status.
	//
	static StatusTypeDef ADC_ConversionStop(ADC_HandleTypeDef* hadc)
	{
		uint32_t tickstart = 0U;

		// Verification if ADC is not already stopped on regular group to bypass
		// this function if not needed.
		if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc))
		{

			// Stop potential conversion on going on regular group
			// Software is allowed to set ADSTP only when ADSTART=1 and ADDIS=0
			if (isBitSet(hadc->Instance->CR, ADC_CR_ADSTART) &&
				IS_BIT_CLR(hadc->Instance->CR, ADC_CR_ADDIS)                  )
			{
				// Stop conversions on regular group
				hadc->Instance->CR |= ADC_CR_ADSTP;
			}

			// Wait for conversion effectively stopped
			// Get tick count
			tickstart = Timers::GetTick();

			while((hadc->Instance->CR & ADC_CR_ADSTART) != RESET)
			{
				if((Timers::GetTick() - tickstart) > ADC_STOP_CONVERSION_TIMEOUT)
				{
					// Update ADC state machine to error
					SET_BIT(hadc->State, ADC_STATE_ERROR_INTERNAL);

					// Set ADC error code to ADC IP internal error
					SET_BIT(hadc->ErrorCode, ADC_ERROR_INTERNAL);

					return STATUS_ERROR;
				}
			}

		}

		// Return status
		return STATUS_OK;
	}

	//
	// @brief  Disable the selected ADC.
	// @note   Prerequisite condition to use this function: ADC conversions must be
	//         stopped.
	// @param  hadc: ADC handle
	// @retval status.
	///
	static StatusTypeDef ADC_Disable(ADC_HandleTypeDef* hadc)
	{
		uint32_t tickstart = 0U;

		// Verification if ADC is not already disabled:
		// Note: forbidden to disable ADC (set bit ADC_CR_ADDIS) if ADC is already
		//       disabled.
		if (ADC_IS_ENABLE(hadc) != RESET)
		{
			// Check if conditions to disable the ADC are fulfilled
			if (ADC_DISABLING_CONDITIONS(hadc) != RESET)
			{
				// Disable the ADC peripheral
				ADC_DISABLE(hadc);
			}
			else
			{
				// Update ADC state machine to error
				SET_BIT(hadc->State, ADC_STATE_ERROR_INTERNAL);

				// Set ADC error code to ADC IP internal error
				SET_BIT(hadc->ErrorCode, ADC_ERROR_INTERNAL);

				return STATUS_ERROR;
			}

			// Wait for ADC effectively disabled
			// Get tick count
			tickstart = Timers::GetTick();

			while(isBitSet(hadc->Instance->CR, ADC_CR_ADEN))
			{
				if((Timers::GetTick() - tickstart) > ADC_DISABLE_TIMEOUT)
				{
					// Update ADC state machine to error
					SET_BIT(hadc->State, ADC_STATE_ERROR_INTERNAL);

					// Set ADC error code to ADC IP internal error
					SET_BIT(hadc->ErrorCode, ADC_ERROR_INTERNAL);

					return STATUS_ERROR;
				}
			}
		}

		// Return status
		return STATUS_OK;
	}

	//
	// @brief ADC MSP de-initialization
	//        This function frees the hardware resources used in this example:
	//          - Disable clock of ADC peripheral
	//          - Revert GPIO associated to the peripheral channels to their default state
	//          - Revert NVIC associated to the peripheral interruptions to its default state
	// @param hadc: ADC handle pointer
	// @retval None
	//
	void ADC_MspDeInit(ADC_HandleTypeDef *hadc)
	{
		//##-1- Reset peripherals ##################################################
		ADC1_FORCE_RESET();
		ADC1_RELEASE_RESET();

		//##-2- Disable peripherals and GPIO Clocks ################################
		// De-initialize GPIO pin of the selected ADC channel
		GPIO::GpioDeInit(ADC1_CHANNEL_VOLTAGE_GPIO_PORT, ADC1_CHANNEL_VOLTAGE_PIN);
		GPIO::GpioDeInit(ADC1_CHANNEL_TEMPERATURE_GPIO_PORT, ADC1_CHANNEL_TEMPERATURE_PIN);

		//##-4- Disable the NVIC ###################################################
		// Disable the NVIC configuration for ADC interrupt
		NVIC_DisableIRQ(ADC1_IRQn);
	}

	//
	// @brief  Deinitialize the ADC peripheral registers to their default reset
	//         values, with deinitialization of the ADC MSP.
	// @note   For devices with several ADCs: reset of ADC common registers is done
	//         only if all ADCs sharing the same common group are disabled.
	//         If this is not the case, reset of these common parameters reset is
	//         bypassed without error reporting: it can be the intended behavior in
	//         case of reset of a single ADC while the other ADCs sharing the same
	//         common group is still running.
	// @param  hadc: ADC handle
	// @retval HAL status
	//
	StatusTypeDef ADC_DeInit(ADC_HandleTypeDef* hadc)
	{
		StatusTypeDef tmp_hal_status = STATUS_OK;

		// Check ADC handle
		if(hadc == NULL)
		{
			return STATUS_ERROR;
		}

		// Set ADC state
		SET_BIT(hadc->State, ADC_STATE_BUSY_INTERNAL);

		// Stop potential conversion on going, on regular group
		tmp_hal_status = ADC_ConversionStop(hadc);

		// Disable ADC peripheral if conversions are effectively stopped
		if (tmp_hal_status == STATUS_OK)
		{
			// Disable the ADC peripheral
			tmp_hal_status = ADC_Disable(hadc);

			// Check if ADC is effectively disabled
			if (tmp_hal_status != STATUS_ERROR)
			{
				// Change ADC state
				hadc->State = ADC_STATE_READY;
			}
		}


		// Configuration of ADC parameters if previous preliminary actions are
		// correctly completed.
		if (tmp_hal_status != STATUS_ERROR)
		{

			// ========== Reset ADC registers ==========
			// Reset register IER
			ADC_DISABLE_IT(hadc, (ADC_IT_AWD | ADC_IT_OVR | ADC_IT_EOCAL | ADC_IT_EOS |  \
				ADC_IT_EOC | ADC_IT_RDY | ADC_IT_EOSMP ));


			// Reset register ISR
			ADC_CLEAR_FLAG(hadc, (ADC_FLAG_AWD | ADC_FLAG_EOCAL | ADC_FLAG_OVR | ADC_FLAG_EOS |  \
				ADC_FLAG_EOC | ADC_FLAG_EOSMP | ADC_FLAG_RDY));


			// Reset register CR
			// Disable voltage regulator
			// Note: Regulator disable useful for power saving
			// Reset ADVREGEN bit
			hadc->Instance->CR &= ~ADC_CR_ADVREGEN;

			// Bits ADC_CR_ADSTP, ADC_CR_ADSTART are in access mode "read-set": no direct reset applicable
			// No action

			// Reset register CFGR1
			hadc->Instance->CFGR1 &= ~(ADC_CFGR1_AWDCH  | ADC_CFGR1_AWDEN  | ADC_CFGR1_AWDSGL | \
			    ADC_CFGR1_DISCEN | ADC_CFGR1_AUTOFF | ADC_CFGR1_AUTDLY | \
			    ADC_CFGR1_CONT   | ADC_CFGR1_OVRMOD | ADC_CFGR1_EXTEN  | \
			    ADC_CFGR1_EXTSEL | ADC_CFGR1_ALIGN  | ADC_CFGR1_RES    | \
			    ADC_CFGR1_SCANDIR| ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN);

			// Reset register CFGR2
			hadc->Instance->CFGR2 &= ~(ADC_CFGR2_TOVS  | ADC_CFGR2_OVSS  | ADC_CFGR2_OVSR | \
			    ADC_CFGR2_OVSE  | ADC_CFGR2_CKMODE );


			// Reset register SMPR
			hadc->Instance->SMPR &= ~(ADC_SMPR_SMPR);

			// Reset register TR
			hadc->Instance->TR &= ~(ADC_TR_LT | ADC_TR_HT);

			// Reset register CALFACT
			hadc->Instance->CALFACT &= ~(ADC_CALFACT_CALFACT);





			// Reset register DR
			// bits in access mode read only, no direct reset applicable

			// Reset register CALFACT
			hadc->Instance->CALFACT &= ~(ADC_CALFACT_CALFACT);

			// ========== Hard reset ADC peripheral ==========
			// Performs a global reset of the entire ADC peripheral: ADC state is
			// forced to a similar state after device power-on.
			// If needed, copy-paste and uncomment the following reset code into
			// function "void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)":
			//
			//  __HAL_RCC_ADC1_FORCE_RESET()
			//  __HAL_RCC_ADC1_RELEASE_RESET()

			// DeInit the low level hardware
			ADC_MspDeInit(hadc);

			// Set ADC error code to none
			ADC_CLEAR_ERRORCODE(hadc);

			// Set ADC state
			hadc->State = ADC_STATE_RESET;
		}

		// Process unlocked
		HANDLE_UNLOCK(hadc);

		// Return function status
		return tmp_hal_status;
	}

	void ADC1_CHANNEL_VOLTAGE_GPIO_CLK_ENABLE() {
		Rcc::RCC_GPIOB_CLK_ENABLE();
	}
	void ADC1_CHANNEL_TEMPERATURE_GPIO_CLK_ENABLE() {
		Rcc::RCC_GPIOA_CLK_ENABLE();
	}

	//
	// @brief ADC MSP initialization
	//        This function configures the hardware resources used in this example:
	//          - Enable clock of ADC peripheral
	//          - Configure the GPIO associated to the peripheral channels
	//          - Configure the NVIC associated to the peripheral interruptions
	// @param hadc: ADC handle pointer
	// @retval None
	//
	void ADC_MspInit(ADC_HandleTypeDef *hadc)
	{
		GPIO::GpioInit_t          GPIO_InitStruct;
//		static DMA_HandleTypeDef  DmaHandle;
		Rcc::RCC_OscInitTypeDef        RCC_OscInitStructure;

		//##-1- Enable peripherals and GPIO Clocks #################################
		// Enable clock of GPIO associated to the peripheral channels
		ADC1_CHANNEL_VOLTAGE_GPIO_CLK_ENABLE();
		ADC1_CHANNEL_TEMPERATURE_GPIO_CLK_ENABLE();

		// Enable clock of ADCx peripheral
		ADC1_CLK_ENABLE();

		// Note: STM32L0xx ADC is using a dedicated asynchronous clock derived
		//       from HSI RC oscillator 16MHz.
		//       The clock source has to be enabled at RCC top level using function
		//       "HAL_RCC_OscConfig()" (see comments in stm32l0xx_hal_adc.c header)

		// Enable asynchronous clock source of ADCx
		Rcc::RCC_GetOscConfig(&RCC_OscInitStructure);
		RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSI;
		RCC_OscInitStructure.HSICalibrationValue = 0x10;
		RCC_OscInitStructure.HSIState = RCC_HSI_ON;
		Rcc::RCC_OscConfig(&RCC_OscInitStructure);

		//##-2a- Configure peripheral GPIO ##########################################
		// Configure GPIO pin of the selected ADC channel
		GPIO_InitStruct.Pin = ADC1_CHANNEL_VOLTAGE_PIN;
		GPIO_InitStruct.Mode = GPIO::GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO::GpioInit(ADC1_CHANNEL_VOLTAGE_GPIO_PORT, &GPIO_InitStruct);

		//##-2b- Configure peripheral GPIO ##########################################
		// Configure GPIO pin of the selected ADC channel
		GPIO_InitStruct.Pin = ADC1_CHANNEL_TEMPERATURE_PIN;
		GPIO_InitStruct.Mode = GPIO::GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO::GpioInit(ADC1_CHANNEL_TEMPERATURE_GPIO_PORT, &GPIO_InitStruct);

		//##-4- Configure the NVIC #################################################

		// NVIC configuration for ADC interrupt
		// Priority: high-priority
		NVIC_SetPriority(ADC1_IRQn, 0);
		NVIC_EnableIRQ(ADC1_IRQn);
	}

	//
	// @brief  Initialize the ADC peripheral and regular group according to
	//         parameters specified in structure "ADC_InitTypeDef".
	// @note   As prerequisite, ADC clock must be configured at RCC top level
	//         depending on possible clock sources: APB clock of HSI clock.
	//         See commented example code below that can be copied and uncommented
	//         into HAL_ADC_MspInit().
	// @note   Possibility to update parameters on the fly:
	//         This function initializes the ADC MSP (HAL_ADC_MspInit()) only when
	//         coming from ADC state reset. Following calls to this function can
	//         be used to reconfigure some parameters of ADC_InitTypeDef
	//         structure on the fly, without modifying MSP configuration. If ADC
	//         MSP has to be modified again, HAL_ADC_DeInit() must be called
	//         before HAL_ADC_Init().
	//         The setting of these parameters is conditioned to ADC state.
	//         For parameters constraints, see comments of structure
	//         "ADC_InitTypeDef".
	// @note   This function configures the ADC within 2 scopes: scope of entire
	//         ADC and scope of regular group. For parameters details, see comments
	//         of structure "ADC_InitTypeDef".
	// @note   When device is in mode low-power (low-power run, low-power sleep or stop mode),
	//         function "HAL_ADCEx_EnableVREFINT()" must be called before function HAL_ADC_Init()
	//         (in case of previous ADC operations: function HAL_ADC_DeInit() must be called first).
	//         In case of internal temperature sensor to be measured:
	//         function "HAL_ADCEx_EnableVREFINTTempSensor()" must be called similarilly.
	// @param  hadc: ADC handle
	// @retval status
	//
	StatusTypeDef ADC_Init(ADC_HandleTypeDef* hadc)
	{

		// Check ADC handle
		if(hadc == NULL)
		{
			return STATUS_ERROR;
		}

		// As prerequisite, into HAL_ADC_MspInit(), ADC clock must be configured
		// at RCC top level depending on both possible clock sources:
		// APB clock or HSI clock.
		// Refer to header of this file for more details on clock enabling procedure

		// Actions performed only if ADC is coming from state reset:
		// - Initialization of ADC MSP
		// - ADC voltage regulator enable
		if(hadc->State == ADC_STATE_RESET)
		{
			// Initialize ADC error code
			ADC_CLEAR_ERRORCODE(hadc);

			// Allocate lock resource and initialize it
			hadc->Lock = HANDLE_UNLOCKED;

			// Init the low level hardware
			ADC_MspInit(hadc);
		}

		// Configuration of ADC parameters if previous preliminary actions are
		// correctly completed.
		// and if there is no conversion on going on regular group (ADC can be
		// enabled anyway, in case of call of this function to update a parameter
		// on the fly).
		if (isBitSet(hadc->State, ADC_STATE_ERROR_INTERNAL) ||
			(ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) != RESET)  )
		{
			// Update ADC state machine to error
			SET_BIT(hadc->State, ADC_STATE_ERROR_INTERNAL);

			// Process unlocked
			HANDLE_UNLOCK(hadc);
			return STATUS_ERROR;
		}

		// Set ADC state
		ADC_STATE_CLR_SET(hadc->State,
			ADC_STATE_REG_BUSY,
			ADC_STATE_BUSY_INTERNAL);

		// Parameters update conditioned to ADC state:
		// Parameters that can be updated only when ADC is disabled:
		//  - ADC clock mode
		//  - ADC clock prescaler
		//  - ADC Resolution
		if (ADC_IS_ENABLE(hadc) == RESET)
		{
			// Some parameters of this register are not reset, since they are set
			// by other functions and must be kept in case of usage of this
			// function on the fly (update of a parameter of ADC_InitTypeDef
			// without needing to reconfigure all other ADC groups/channels
			// parameters):
			//   - internal measurement paths: Vbat, temperature sensor, Vref
			//     (set into HAL_ADC_ConfigChannel() )

			// Configuration of ADC clock: clock source PCLK or asynchronous with
			// selectable prescaler
			ADC_CLOCK_PRESCALER(hadc);

			// Configuration of ADC:
			//  - Resolution
			hadc->Instance->CFGR1 &= ~( ADC_CFGR1_RES);
			hadc->Instance->CFGR1 |= hadc->Init.Resolution;
		}

		// Set the Low Frequency mode
		ADC->CCR &= (uint32_t)~ADC_CCR_LFMEN;
		ADC->CCR |= ADC_CCR_LOWFREQUENCY(hadc->Init.LowPowerFrequencyMode);

		// Enable voltage regulator (if disabled at this step)
		if (IS_BIT_CLR(hadc->Instance->CR, ADC_CR_ADVREGEN))
		{
			// Set ADVREGEN bit
			hadc->Instance->CR |= ADC_CR_ADVREGEN;
		}

		// Configuration of ADC:
		//  - Resolution
		//  - Data alignment
		//  - Scan direction
		//  - External trigger to start conversion
		//  - External trigger polarity
		//  - Continuous conversion mode
		//  - DMA continuous request
		//  - Overrun
		//  - AutoDelay feature
		//  - Discontinuous mode
		hadc->Instance->CFGR1 &= ~(ADC_CFGR1_ALIGN   |
		    ADC_CFGR1_SCANDIR |
			ADC_CFGR1_EXTSEL  |
			ADC_CFGR1_EXTEN   |
			ADC_CFGR1_CONT    |
			ADC_CFGR1_DMACFG  |
			ADC_CFGR1_OVRMOD  |
			ADC_CFGR1_AUTDLY  |
			ADC_CFGR1_AUTOFF  |
			ADC_CFGR1_DISCEN
		);

		hadc->Instance->CFGR1 |= (hadc->Init.DataAlign                             |
			ADC_SCANDIR(hadc->Init.ScanConvMode)             |
			ADC_CONTINUOUS(hadc->Init.ContinuousConvMode)    |
			ADC_DMACONTREQ(hadc->Init.DMAContinuousRequests) |
			hadc->Init.Overrun                               |
			ADC_CFGR1_AutoDelay(hadc->Init.LowPowerAutoWait) |
			ADC_CFGR1_AUTO_OFF(hadc->Init.LowPowerAutoPowerOff));

		// Enable external trigger if trigger selection is different of software
		// start.
		// Note: This configuration keeps the hardware feature of parameter
		//       ExternalTrigConvEdge "trigger edge none" equivalent to
		//       software start.
		if (hadc->Init.ExternalTrigConv != ADC_SOFTWARE_START)
		{
			hadc->Instance->CFGR1 |= hadc->Init.ExternalTrigConv |
			    hadc->Init.ExternalTrigConvEdge;
		}

		// Enable discontinuous mode only if continuous mode is disabled
		if (hadc->Init.DiscontinuousConvMode == ENABLE)
		{
			if (hadc->Init.ContinuousConvMode == DISABLE)
			{
				// Enable the selected ADC group regular discontinuous mode
				hadc->Instance->CFGR1 |= (ADC_CFGR1_DISCEN);
			}
			else
			{
				// ADC regular group discontinuous was intended to be enabled,
				// but ADC regular group modes continuous and sequencer discontinuous
				// cannot be enabled simultaneously.

				// Update ADC state machine to error
				SET_BIT(hadc->State, ADC_STATE_ERROR_CONFIG);

				// Set ADC error code to ADC IP internal error
				SET_BIT(hadc->ErrorCode, ADC_ERROR_INTERNAL);
			}
		}

		if (hadc->Init.OversamplingMode == ENABLE)
		{
			// Configuration of Oversampler:
			//  - Oversampling Ratio
			//  - Right bit shift
			//  - Triggered mode

			hadc->Instance->CFGR2 &= ~( ADC_CFGR2_OVSR |
				ADC_CFGR2_OVSS |
				ADC_CFGR2_TOVS );

			hadc->Instance->CFGR2 |= ( hadc->Init.Oversample.Ratio         |
				hadc->Init.Oversample.RightBitShift             |
				hadc->Init.Oversample.TriggeredMode );

			// Enable OverSampling mode
			hadc->Instance->CFGR2 |= ADC_CFGR2_OVSE;
		}
		else
		{
			if(isBitSet(hadc->Instance->CFGR2, ADC_CFGR2_OVSE))
			{
				// Disable OverSampling mode if needed
				hadc->Instance->CFGR2 &= ~ADC_CFGR2_OVSE;
			}
		}

		// Clear the old sampling time
		hadc->Instance->SMPR &= (uint32_t)(~ADC_SMPR_SMPR);

		// Set the new sample time
		hadc->Instance->SMPR |= hadc->Init.SamplingTime;

		// Clear ADC error code
		ADC_CLEAR_ERRORCODE(hadc);

		// Set the ADC state
		ADC_STATE_CLR_SET(hadc->State,
			ADC_STATE_BUSY_INTERNAL,
			ADC_STATE_READY);


		// Return function status
		return STATUS_OK;
	}

	//
	// @brief  Delay micro seconds
	// @param  microSecond : delay
	// @retval None
	//
	void ADC_DelayMicroSecond(uint32_t microSecond)
	{
		// Compute number of CPU cycles to wait for
		__IO uint32_t waitLoopIndex = (microSecond * (SystemCoreClock / 1000000U));

		while(waitLoopIndex != 0U)
		{
			waitLoopIndex--;
		}
	}

	//
	// @brief  Configure a channel to be assigned to ADC group regular.
	// @note   In case of usage of internal measurement channels:
	//         VrefInt/Vlcd(STM32L0x3xx only)/TempSensor.
	//         Sampling time constraints must be respected (sampling time can be
	//         adjusted in function of ADC clock frequency and sampling time
	//         setting).
	//         Refer to device datasheet for timings values, parameters TS_vrefint,
	//         TS_vlcd (STM32L0x3xx only), TS_temp (values rough order: 5us to 17us).
	//         These internal paths can be be disabled using function
	//         HAL_ADC_DeInit().
	// @note   Possibility to update parameters on the fly:
	//         This function initializes channel into ADC group regular,
	//         following calls to this function can be used to reconfigure
	//         some parameters of structure "ADC_ChannelConfTypeDef" on the fly,
	//         without resetting the ADC.
	//         The setting of these parameters is conditioned to ADC state:
	//         Refer to comments of structure "ADC_ChannelConfTypeDef".
	// @param  hadc: ADC handle
	// @param  sConfig: Structure of ADC channel assigned to ADC group regular.
	// @retval status
	//
	StatusTypeDef ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig)
	{
		// Process locked
		HANDLE_LOCK(hadc);

		// Parameters update conditioned to ADC state:
		// Parameters that can be updated when ADC is disabled or enabled without
		// conversion on going on regular group:
		//  - Channel number
		//  - Management of internal measurement channels: Vbat/VrefInt/TempSensor
		if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) != RESET)
		{
			// Update ADC state machine to error
			SET_BIT(hadc->State, ADC_STATE_ERROR_CONFIG);
			// Process unlocked
			HANDLE_UNLOCK(hadc);
			return STATUS_ERROR;
		}

		if (sConfig->Rank != ADC_RANK_NONE)
		{
			// Enable selected channels
			hadc->Instance->CHSELR |= (uint32_t)(sConfig->Channel & ADC_CHANNEL_MASK);

			// Management of internal measurement channels: Vlcd (STM32L0x3xx only)/VrefInt/TempSensor
			// internal measurement paths enable: If internal channel selected, enable
			// dedicated internal buffers and path.

			// If Temperature sensor channel is selected, then enable the internal
			// buffers and path
			if (((sConfig->Channel & ADC_CHANNEL_MASK) & ADC_CHANNEL_TEMPSENSOR ) == (ADC_CHANNEL_TEMPSENSOR & ADC_CHANNEL_MASK))
			{
				ADC->CCR |= ADC_CCR_TSEN;

				// Delay for temperature sensor stabilization time
				ADC_DelayMicroSecond(ADC_TEMPSENSOR_DELAY_US);
			}

			// If VRefInt channel is selected, then enable the internal buffers and path
			if (((sConfig->Channel & ADC_CHANNEL_MASK) & ADC_CHANNEL_VREFINT) == (ADC_CHANNEL_VREFINT & ADC_CHANNEL_MASK))
			{
				ADC->CCR |= ADC_CCR_VREFEN;
			}

			#if defined (STM32L053xx) || defined (STM32L063xx) || defined (STM32L073xx) || defined (STM32L083xx)
				// If Vlcd channel is selected, then enable the internal buffers and path
				if (((sConfig->Channel & ADC_CHANNEL_MASK) & ADC_CHANNEL_VLCD) == (ADC_CHANNEL_VLCD & ADC_CHANNEL_MASK))
				{
					ADC->CCR |= ADC_CCR_VLCDEN;
				}
			#endif
		}
		else
		{
			// Regular sequence configuration
			// Reset the channel selection register from the selected channel
			hadc->Instance->CHSELR &= ~((uint32_t)(sConfig->Channel & ADC_CHANNEL_MASK));

			// Management of internal measurement channels: VrefInt/TempSensor/Vbat
			// internal measurement paths disable: If internal channel selected,
			// disable dedicated internal buffers and path.
			if (((sConfig->Channel & ADC_CHANNEL_MASK) & ADC_CHANNEL_TEMPSENSOR ) == (ADC_CHANNEL_TEMPSENSOR & ADC_CHANNEL_MASK))
			{
				ADC->CCR &= ~ADC_CCR_TSEN;
			}

			// If VRefInt channel is selected, then enable the internal buffers and path
			if (((sConfig->Channel & ADC_CHANNEL_MASK) & ADC_CHANNEL_VREFINT) == (ADC_CHANNEL_VREFINT & ADC_CHANNEL_MASK))
			{
				ADC->CCR &= ~ADC_CCR_VREFEN;
			}

			#if defined (STM32L053xx) || defined (STM32L063xx) || defined (STM32L073xx) || defined (STM32L083xx)
				// If Vlcd channel is selected, then enable the internal buffers and path
				if (((sConfig->Channel & ADC_CHANNEL_MASK) & ADC_CHANNEL_VLCD) == (ADC_CHANNEL_VLCD & ADC_CHANNEL_MASK))
				{
					ADC->CCR &= ~ADC_CCR_VLCDEN;
				}
			#endif
		}

		// Process unlocked
		HANDLE_UNLOCK(hadc);

		// Return function status
		return STATUS_OK;
	}

	//
	// @brief  Enable the selected ADC.
	// @note   Prerequisite condition to use this function: ADC must be disabled
	//         and voltage regulator must be enabled (done into HAL_ADC_Init()).
	// @note   If low power mode AutoPowerOff is enabled, power-on/off phases are
	//         performed automatically by hardware.
	//         In this mode, this function is useless and must not be called because
	//         flag ADC_FLAG_RDY is not usable.
	//         Therefore, this function must be called under condition of
	//         "if (hadc->Init.LowPowerAutoPowerOff != ENABLE)".
	// @param  hadc: ADC handle
	// @retval status.
	//
	StatusTypeDef ADC_Enable(ADC_HandleTypeDef* hadc)
	{
		uint32_t tickstart = 0U;

		// ADC enable and wait for ADC ready (in case of ADC is disabled or
		// enabling phase not yet completed: flag ADC ready not yet set).
		// Timeout implemented to not be stuck if ADC cannot be enabled (possible
		// causes: ADC clock not running, ...).
		if (ADC_IS_ENABLE(hadc) == RESET)
		{
			// Check if conditions to enable the ADC are fulfilled
			if (ADC_ENABLING_CONDITIONS(hadc) == RESET)
			{
				// Update ADC state machine to error
				SET_BIT(hadc->State, ADC_STATE_ERROR_INTERNAL);

				// Set ADC error code to ADC IP internal error
				SET_BIT(hadc->ErrorCode, ADC_ERROR_INTERNAL);

				return STATUS_ERROR;
			}

			// Enable the ADC peripheral
			ADC_ENABLE(hadc);

			// Delay for ADC stabilization time.
			ADC_DelayMicroSecond(ADC_STAB_DELAY_US);

			// Get tick count
			tickstart = Timers::GetTick();

			// Wait for ADC effectively enabled
			while(ADC_GET_FLAG(hadc, ADC_FLAG_RDY) == RESET)
			{
				if((Timers::GetTick() - tickstart) > ADC_ENABLE_TIMEOUT)
				{
					// Update ADC state machine to error
					SET_BIT(hadc->State, ADC_STATE_ERROR_INTERNAL);

					// Set ADC error code to ADC IP internal error
					SET_BIT(hadc->ErrorCode, ADC_ERROR_INTERNAL);

					return STATUS_ERROR;
				}
			}
		}

		// Return status
		return STATUS_OK;
	}

	//
	// @brief  Conversion complete callback in non blocking mode
	// @param  AdcHandle : ADC handle
	// @note   This example shows a simple way to report end of conversion
	//         and get conversion result. You can add your own implementation.
	// @retval None
	//
	void ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
	{
		// Report to main program that ADC sequencer has reached its end
		ubSequenceCompleted = SET;
	}

	//
	// @brief  Conversion DMA half-transfer callback in non blocking mode
	// @param  hadc: ADC handle
	// @retval None
	//
	void ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
	{

	}

	//
	// @brief  ADC error callback in non blocking mode
	//        (ADC conversion with interruption or transfer by DMA)
	// @param  hadc: ADC handle
	// @retval None
	//
	void ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
	{
		// In case of ADC error, call main error handler
		Thermoprinter::Error_Handler(__FILE__, __LINE__);
	}

	//
	// @brief  DMA transfer complete callback.
	// @param  hdma: pointer to DMA handle.
	// @retval None
	//
	static void ADC_DMAConvCplt(DMA_HandleTypeDef *hdma)
	{
		// Retrieve ADC handle corresponding to current DMA handle
		ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

		// Update state machine on conversion status if not in error state
		if (IS_BIT_CLR(hadc->State, ADC_STATE_ERROR_INTERNAL | ADC_STATE_ERROR_DMA))
		{
			// Set ADC state
			SET_BIT(hadc->State, ADC_STATE_REG_EOC);

			// Determine whether any further conversion upcoming on group regular
			// by external trigger, continuous mode or scan sequence on going.
			if(ADC_IS_SOFTWARE_START_REGULAR(hadc)        &&
				(hadc->Init.ContinuousConvMode == DISABLE)   )
			{
				// If End of Sequence is reached, disable interrupts
				if( ADC_GET_FLAG(hadc, ADC_FLAG_EOS) )
				{
					// Allowed to modify bits ADC_IT_EOC/ADC_IT_EOS only if bit
					// ADSTART==0 (no conversion on going)
					if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET)
					{
						// Disable ADC end of single conversion interrupt on group regular
						// Note: Overrun interrupt was enabled with EOC interrupt in
						// HAL_Start_IT(), but is not disabled here because can be used
						// by overrun IRQ process below.
						ADC_DISABLE_IT(hadc, ADC_IT_EOC | ADC_IT_EOS);

						// Set ADC state
						ADC_STATE_CLR_SET(hadc->State,
							ADC_STATE_REG_BUSY,
							ADC_STATE_READY);
					}
					else
					{
						// Change ADC state to error state
						SET_BIT(hadc->State, ADC_STATE_ERROR_CONFIG);

						// Set ADC error code to ADC IP internal error
						SET_BIT(hadc->ErrorCode, ADC_ERROR_INTERNAL);
					}
				}
			}

			// Conversion complete callback
			ADC_ConvCpltCallback(hadc);
		}
		else
		{
			// Call DMA error callback
			hadc->DMA_Handle->XferErrorCallback(hdma);
		}
	}

	//
	// @brief  DMA half transfer complete callback.
	// @param  hdma: pointer to DMA handle.
	// @retval None
	//
	static void ADC_DMAHalfConvCplt(DMA_HandleTypeDef *hdma)
	{
		// Retrieve ADC handle corresponding to current DMA handle
		ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

		// Half conversion callback
		ADC_ConvHalfCpltCallback(hadc);
	}

	//
	// @brief  DMA error callback.
	// @param  hdma: pointer to DMA handle.
	// @retval None
	//
	static void ADC_DMAError(DMA_HandleTypeDef *hdma)
	{
		// Retrieve ADC handle corresponding to current DMA handle
		ADC_HandleTypeDef* hadc = ( ADC_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

		// Set ADC state
		SET_BIT(hadc->State, ADC_STATE_ERROR_DMA);

		// Set ADC error code to DMA error
		SET_BIT(hadc->ErrorCode, ADC_ERROR_DMA);

		// Error callback
		ADC_ErrorCallback(hadc);
	}

	//
	// @brief  Sets the DMA Transfer parameter.
	// @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
	//                     the configuration information for the specified DMA Channel.
	// @param  SrcAddress: The source memory Buffer address
	// @param  DstAddress: The destination memory Buffer address
	// @param  DataLength: The length of data to be transferred from source to destination
	// @retval HAL status
	//
	static void DMA_SetConfig(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
	{
		// Configure DMA Channel data length
		hdma->Instance->CNDTR = DataLength;

		// Peripheral to Memory
		if((hdma->Init.Direction) == DMA_MEMORY_TO_PERIPH)
		{
			// Configure DMA Channel destination address
			hdma->Instance->CPAR = DstAddress;

			// Configure DMA Channel source address
			hdma->Instance->CMAR = SrcAddress;
		}
		// Memory to Peripheral
		else
		{
			// Configure DMA Channel source address
			hdma->Instance->CPAR = SrcAddress;

			// Configure DMA Channel destination address
			hdma->Instance->CMAR = DstAddress;
		}
	}

	//
	// @brief  Start the DMA Transfer with interrupt enabled.
	// @param  hdma:       pointer to a DMA_HandleTypeDef structure that contains
	//                     the configuration information for the specified DMA Channel.
	// @param  SrcAddress: The source memory Buffer address
	// @param  DstAddress: The destination memory Buffer address
	// @param  DataLength: The length of data to be transferred from source to destination
	// @retval status
	//
	StatusTypeDef DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength)
	{
		// Process locked
		HANDLE_LOCK(hdma);

		// Change DMA peripheral state
		hdma->State = DMA_STATE_BUSY;

		// Disable the peripheral
		DMA_DISABLE(hdma);

		// Configure the source, destination address and the data length
		DMA_SetConfig(hdma, SrcAddress, DstAddress, DataLength);

		// Enable the transfer complete interrupt
		DMA_ENABLE_IT(hdma, DMA_IT_TC);

		// Enable the Half transfer complete interrupt
		DMA_ENABLE_IT(hdma, DMA_IT_HT);

		// Enable the transfer Error interrupt
		DMA_ENABLE_IT(hdma, DMA_IT_TE);

		// Enable the Peripheral
		DMA_ENABLE(hdma);

		return STATUS_OK;
	}

	//
	// @brief  Enable ADC, start conversion of regular group and transfer result through DMA.
	// @note   Interruptions enabled in this function:
	//         overrun (if applicable), DMA half transfer, DMA transfer complete.
	//         Each of these interruptions has its dedicated callback function.
	// @param  hadc: ADC handle
	// @param  pData: Destination Buffer address.
	// @param  Length: Length of data to be transferred from ADC peripheral to memory (in bytes)
	// @retval status.
	//
	StatusTypeDef ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length)
	{
		StatusTypeDef tmp_hal_status = STATUS_OK;

		// Perform ADC enable and conversion start if no conversion is on going
		if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET)
		{
			// Process locked
			HANDLE_LOCK(hadc);

			// Enable the ADC peripheral
			// If low power mode AutoPowerOff is enabled, power-on/off phases are
			// performed automatically by hardware.
			if (hadc->Init.LowPowerAutoPowerOff != ENABLE)
			{
				tmp_hal_status = ADC_Enable(hadc);
			}

			// Start conversion if ADC is effectively enabled
			if (tmp_hal_status == STATUS_OK)
			{
				// Set ADC state
				// - Clear state bitfield related to regular group conversion results
				// - Set state bitfield related to regular operation
				ADC_STATE_CLR_SET(hadc->State,
					ADC_STATE_READY | ADC_STATE_REG_EOC | ADC_STATE_REG_OVR | ADC_STATE_REG_EOSMP,
					ADC_STATE_REG_BUSY);

				// Reset ADC all error code fields
				ADC_CLEAR_ERRORCODE(hadc);

				// Process unlocked
				// Unlock before starting ADC conversions: in case of potential
				// interruption, to let the process to ADC IRQ Handler.
				HANDLE_UNLOCK(hadc);

				// Set the DMA transfer complete callback
				hadc->DMA_Handle->XferCpltCallback = ADC_DMAConvCplt;

				// Set the DMA half transfer complete callback
				hadc->DMA_Handle->XferHalfCpltCallback = ADC_DMAHalfConvCplt;

				// Set the DMA error callback
				hadc->DMA_Handle->XferErrorCallback = ADC_DMAError;


				// Manage ADC and DMA start: ADC overrun interruption, DMA start, ADC
				// start (in case of SW start):

				// Clear regular group conversion flag and overrun flag
				// (To ensure of no unknown state from potential previous ADC
				// operations)
				ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

				// Enable ADC overrun interrupt
				ADC_ENABLE_IT(hadc, ADC_IT_OVR);

				// Enable ADC DMA mode
				hadc->Instance->CFGR1 |= ADC_CFGR1_DMAEN;

				// Start the DMA channel
				DMA_Start_IT(hadc->DMA_Handle, (uint32_t)&hadc->Instance->DR, (uint32_t)pData, Length);

				// Enable conversion of regular group.
				// If software start has been selected, conversion starts immediately.
				// If external trigger has been selected, conversion will start at next
				// trigger event.
				hadc->Instance->CR |= ADC_CR_ADSTART;
			}
		}
		else
		{
			tmp_hal_status = STATUS_BUSY;
		}

		// Return function status
		return tmp_hal_status;
	}

	//
	// @brief  ADC configuration
	// @param  None
	// @retval None
	//
	void ADC_Config(void)
	{
		ADC_ChannelConfTypeDef   sConfig;

		// Configuration of AdcHandle init structure: ADC parameters and regular group
		AdcHandle.Instance = ADC1;

		if (ADC_DeInit(&AdcHandle) != STATUS_OK)
		{
			// ADC initialization error
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		AdcHandle.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;
		AdcHandle.Init.Resolution            = ADC_RESOLUTION_10B;
		AdcHandle.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
		AdcHandle.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;    // Sequencer will convert the number of channels configured below, successively from the lowest to the highest channel number
		AdcHandle.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
		AdcHandle.Init.LowPowerAutoWait      = DISABLE;
		AdcHandle.Init.LowPowerAutoPowerOff  = DISABLE;
		AdcHandle.Init.ContinuousConvMode    = DISABLE;                       // Continuous mode disabled to have only 1 rank converted at each conversion trig, and because discontinuous mode is enabled
		AdcHandle.Init.DiscontinuousConvMode = ENABLE;                        // Sequencer of regular group will convert the sequence in several sub-divided sequences
		AdcHandle.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T2_TRGO;     // Parameter discarded because trig of conversion without external event
		AdcHandle.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;    // Trig of conversion without external event: start done manually by software
		AdcHandle.Init.DMAContinuousRequests = ENABLE;                        // ADC-DMA continuous requests to match with DMA configured in circular mode
		AdcHandle.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
		AdcHandle.Init.LowPowerFrequencyMode = DISABLE;
		// Note: Set long sampling time due to internal channels (VrefInt,
		//       temperature sensor) constraints. Refer to device datasheet for
		//       min/typ/max values.
		AdcHandle.Init.SamplingTime          = ADC_SAMPLETIME_79CYCLES_5;
		AdcHandle.Init.OversamplingMode      = DISABLE;


		if (ADC_Init(&AdcHandle) != STATUS_OK)
		{
			// ADC initialization error
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		// Configuration of channel on ADCx regular group on sequencer rank 1
		// Note: Considering IT occurring after each ADC conversion (IT by DMA end
		//       of transfer), select sampling time and ADC clock with sufficient
		//       duration to not create an overhead situation in IRQHandler.
		sConfig.Channel      = ADC1_CHANNEL_TEMPERATURE;

		if (ADC_ConfigChannel(&AdcHandle, &sConfig) != STATUS_OK)
		{
			// Channel Configuration Error
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		// Configuration of channel on ADCx regular group on sequencer rank 2
		// Replicate previous rank settings, change only channel
		// Note: On STM32L0xx, rank is defined by channel number. ADC Channel
		//       ADC_CHANNEL_VREFINT is on ADC channel 17, there is 1 other
		//       channel enabled with lower channel number. Therefore,
		//       ADC_CHANNEL_VREFINT will be converted by the sequencer as the
		//       2nd rank.
		sConfig.Channel      = ADC1_CHANNEL_VOLTAGE;

		if (ADC_ConfigChannel(&AdcHandle, &sConfig) != STATUS_OK)
		{
			// Channel Configuration Error
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		//## Start ADC conversions #################################################

		// Start ADC conversion on regular group with transfer by DMA
		if (ADC_Start_DMA(&AdcHandle,
			(uint32_t *)aADCxConvertedValues,
			ADCCONVERTEDVALUES_BUFFER_SIZE
			) != STATUS_OK)
		{
			// Start Error
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		// Run the ADC calibration in single-ended mode
		if (ADCEx_Calibration_Start(&AdcHandle, ADC_SINGLE_ENDED) != STATUS_OK)
		{
			// Calibration Error
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

	}

	//
	// @brief  Enable ADC, start conversion of regular group.
	// @note   Interruptions enabled in this function: None.
	// @param  hadc: ADC handle
	// @retval status
	///
	StatusTypeDef ADC_Start(ADC_HandleTypeDef* hadc)
	{
		StatusTypeDef tmp_hal_status = STATUS_OK;

		// Perform ADC enable and conversion start if no conversion is on going
		if (ADC_IS_CONVERSION_ONGOING_REGULAR(hadc) == RESET)
		{
			// Process locked
			HANDLE_LOCK(hadc);

			// Enable the ADC peripheral
			// If low power mode AutoPowerOff is enabled, power-on/off phases are
			// performed automatically by hardware.
			if (hadc->Init.LowPowerAutoPowerOff != ENABLE)
			{
				tmp_hal_status = ADC_Enable(hadc);
			}

			// Start conversion if ADC is effectively enabled
			if (tmp_hal_status == STATUS_OK)
			{
				// Set ADC state
				// - Clear state bitfield related to regular group conversion results
				// - Set state bitfield related to regular operation
				ADC_STATE_CLR_SET(hadc->State,
					ADC_STATE_READY | ADC_STATE_REG_EOC | ADC_STATE_REG_OVR | ADC_STATE_REG_EOSMP,
					ADC_STATE_REG_BUSY);

				// Reset ADC all error code fields
				ADC_CLEAR_ERRORCODE(hadc);

				// Process unlocked
				// Unlock before starting ADC conversions: in case of potential
				// interruption, to let the process to ADC IRQ Handler.
				HANDLE_UNLOCK(hadc);

				// Clear regular group conversion flag and overrun flag
				// (To ensure of no unknown state from potential previous ADC
				// operations)
				ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));

				// Enable conversion of regular group.
				// If software start has been selected, conversion starts immediately.
				// If external trigger has been selected, conversion will start at next
				// trigger event.
				hadc->Instance->CR |= ADC_CR_ADSTART;
			}
		}
		else
		{
			tmp_hal_status = STATUS_BUSY;
		}

		// Return function status
		return tmp_hal_status;
	}

	void isr() {
		// The stepper ISR can interrupt this ISR. When it does it re-enables this ISR
		// at the end of its run, potentially causing re-entry. This flag prevents it.
		if (in_temp_isr) {
			return;
		}
		in_temp_isr = true;

		// Allow UART and stepper ISRs
		NVIC_DisableIRQ(TIM2_IRQn);
		__enable_irq();

		static int8_t temp_count = -1;
		static ADCSensorState adc_sensor_state = StartupDelay;

		//
		// One sensor is sampled on every other call of the ISR.
		// Each sensor is read 16 (OVERSAMPLENR) times, taking the average.
		//
		// On each Prepare pass, ADC is started for a sensor pin.
		// On the next pass, the ADC value is read and accumulated.
		//
		// This gives each ADC 0.9765ms to charge up.
		//

		switch (adc_sensor_state) {

			case SensorsReady: {
				// All sensors have been read. Stay in this state for a few
				// ISRs to save on calls to temp update/checking code below.
				constexpr int8_t extra_loops = MIN_ADC_ISR_LOOPS - (int8_t)SensorsReady;
				static uint8_t delay_count = 0U;
				if (extra_loops > 0) {
					if (delay_count == 0) {
						delay_count = extra_loops;   // Init this delay
					}
					if (--delay_count) {                                // While delaying...
						adc_sensor_state = (ADCSensorState)(int(SensorsReady) - 1); // retain this state (else, next state will be 0)
					}
				}
				else {
					adc_sensor_state = (ADCSensorState)0; // Fall-through to start first sensor now
				}
				break;
			}

			case PrepareSensors:
				// Start ADC conversion
				// Since sequencer is enabled in discontinuous mode, this will perform
				// the conversion of the next rank in sequencer.
				// Note: For this example, conversion is triggered by software start,
				//       therefore "HAL_ADC_Start()" must be called for each conversion.
				//       Since DMA transfer has been initiated previously by function
				//       "HAL_ADC_Start_DMA()", this function will keep DMA transfer
				//       active.
				if (ADC_Start(&AdcHandle) != STATUS_OK)
				{
					Thermoprinter::Error_Handler(__FILE__, __LINE__);
				}
				break;

			case MeasureSensors:
				if (ubSequenceCompleted == SET)
				{
					// Computation of ADC conversions raw data to physical values
					// Note: ADC results are transferred into array "aADCxConvertedValues"
					//       in the order of their rank in ADC sequencer.
					raw_temp_value += aADCxConvertedValues[0];
					raw_volt_value += aADCxConvertedValues[1];

					// Reset variable for next loop iteration
					ubSequenceCompleted = RESET;
				}
				break;

			case StartupDelay:
			default:
				break;

		} // switch(adc_sensor_state)

		if (!adc_sensor_state && ++temp_count >= OVERSAMPLENR) { // 10 * 16 * 1/(16000000/64/256)  = 164ms.

			temp_count = 0;

			// Update the raw values if they've been read. Else we could be updating them during reading.
			if (!sens_meas_ready) {
				set_current_sens_raw();
			}

			raw_temp_value = 0;
			raw_volt_value = 0;

			#define TEMPDIR() ((PRINTHEAD_RAW_LO_TEMP) > (PRINTHEAD_RAW_HI_TEMP) ? -1 : 1)
			#define VOLTDIR() ((BATTERY_RAW_LO_VOLT) > (BATTERY_RAW_HI_VOLT) ? -1 : 1)

			int32_t constexpr temp_dir = TEMPDIR();
			int32_t constexpr volt_dir = VOLTDIR();

			const int16_t tdir = temp_dir;
			const int16_t rawtemp = current_temperature_raw * tdir;
			if (rawtemp > maxttemp_raw * tdir) {
				max_sens_error();
			}
			if (rawtemp < minttemp_raw * tdir) {
			  min_sens_error();
			}
			const int16_t vdir = volt_dir;
			const int16_t rawvolt = current_voltage_raw * vdir;
			if (rawvolt > maxtvolt_raw * vdir) {
				max_sens_error();
			}
			if (rawvolt < mintvolt_raw * vdir) {
				min_sens_error();
			}
		} // temp_count >= OVERSAMPLENR

		// Go to the next state, up to SensorsReady
		adc_sensor_state = (ADCSensorState)((int(adc_sensor_state) + 1) % int(StartupDelay));

		if (Endstops::e_hit && Endstops::enabled) {
			Endstops::update();  // call endstop update routine
			Endstops::e_hit--;
		}

		__disable_irq();
		in_temp_isr = false;
		NVIC_EnableIRQ(TIM2_IRQn); //re-enable Temperature ISR
	}

	// Derived from RepRap FiveD extruder::getTemperature()
	// For hot end temperature measurement.
	float32_t analog2temp(int32_t raw) {
		if (heater_ttbl_map != NULL) {
			float celsius = 0.F;
			uint8_t i;
			short(*tt)[][2] = (short(*)[][2])(heater_ttbl_map);

			for (i = 1; i < heater_ttbllen_map; i++) {
				if ((short)((*tt)[i][0]) > raw) {
					celsius = (short)((*tt)[i - 1][1]) +
						(raw - (short)((*tt)[i - 1][0])) *
						(float)((short)((*tt)[i][1]) - (short)((*tt)[i - 1][1])) /
						(float)((short)((*tt)[i][0]) - (short)((*tt)[i - 1][0]));
					break;
				}
			}

			// Overflow: Set to last value in the table
			if (i == heater_ttbllen_map) {
				celsius = (short)((*tt)[i - 1][1]);
			}

			return celsius;
		}
		//  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR));
		return ((raw * ((3.3 * 100.0) / 1024.0) / OVERSAMPLENR));
	}

	// For battery voltage measurement.
	float32_t analog2volt(int32_t raw) {
		if (battery_ttbl_map != nullptr) {
			float32_t voltage = 0.F;
			uint8_t i;
			short(*tt)[][2] = (short(*)[][2])(battery_ttbl_map);

			for (i = 1; i < battery_ttbllen_map; i++) {
				if ((short)((*tt)[i][0]) > raw) {
					voltage = (short)((*tt)[i - 1][1]) +
						(raw - (short)((*tt)[i - 1][0])) *
						(float)((short)((*tt)[i][1]) - (short)((*tt)[i - 1][1])) /
						(float)((short)((*tt)[i][0]) - (short)((*tt)[i - 1][0]));
					break;
				}
			}

			// Overflow: Set to last value in the table
			if (i == battery_ttbllen_map) {
				voltage = (short)((*tt)[i - 1][1]);
			}

			return voltage;
		}
		//  return ((raw * ((5.0 * 100.0) / 1024.0) / OVERSAMPLENR));
		return ((raw * ((3.3 * 100.0) / 1024.0) / OVERSAMPLENR));
	}

	//
	// Get raw temperatures
	//
	void set_current_sens_raw() {
		current_temperature_raw = raw_temp_value;
		current_voltage_raw = raw_volt_value;
		sens_meas_ready = true;
	}

	//
	// Temperature Error Handlers
	//
	void _sens_error(const char * const serial_msg) {
		static bool killed = false;
		if (Thermoprinter::IsRunning()) {
			Serial::SERIAL_ERROR_START();
			SERIAL_PROTOCOLPGM(serial_msg);
			SERIAL_ERRORPGM(MSG_STOPPED_HEATER);
		}
		if (!killed) {
			Thermoprinter::Running = false;
			killed = true;
			Thermoprinter::kill();
		}
		else {
			disable_all_heaters(); // paranoia
		}
	}

	void max_sens_error() {
		_sens_error((const char *)(MSG_T_MAXTEMP));
	}
	void min_sens_error() {
		_sens_error((const char *)(MSG_T_MINTEMP));
	}

	void disable_all_heaters() {
	  // for sure our print job has stopped
	}

	// End

}

extern "C" {
	//
	// This ISR uses the compare method so it runs at the base
	// frequency (16 MHz / 64 / 256 = 976.5625 Hz), but at the CNT set
	// in ARR above (128 or halfway between OVFs).
	//
	//  - Prepare or Measure one of the raw ADC sensor values
	//  - Check new temperature values for MIN/MAX errors (kill on error)
	//  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
	//
	void TIM2_IRQHandler(void)
	{
		AdcManager::isr();
	}
}
