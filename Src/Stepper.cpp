/*
 * Stepper.cpp
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

//
//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
//
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.
//

#include <stdint.h>
#include <stm32l0xx.h>
#include "macros.h"
#include "gpio.h"
#include "typedefs.h"
#include "main.h"
#include "Planner.h"
#include "Configuration.h"
#include "Stepper.h"
#include <stddef.h>
#include "Temperature.h"
#include "Endstops.h"
#include "tim.h"
#include "Thermoprinter.h"
#include "Conditionals.h"
#include "rcc.h"

namespace Stepper {
	//
	// Private constants and definitions
	//

	#define MOTOR_DIR_WRITE(STATE) WRITE(MOTOR_DIR,STATE)
	#define MOTOR_STEP_WRITE(STATE) WRITE(MOTOR_STEP,STATE)

	#define MOTOR_APPLY_DIR(v,Q) MOTOR_DIR_WRITE(v)
	#define MOTOR_APPLY_STEP(v,Q) MOTOR_STEP_WRITE(v)

	#define ENABLE_STEPPER_DRIVER_INTERRUPT()  NVIC_EnableIRQ(TIM6_DAC_IRQn);
	#define DISABLE_STEPPER_DRIVER_INTERRUPT() NVIC_DisableIRQ(TIM6_DAC_IRQn);

	uint16_t const speed_lookuptable_fast[256][2] = {
	  { 62500, 55556}, { 6944, 3268}, { 3676, 1176}, { 2500, 607}, { 1893, 369}, { 1524, 249}, { 1275, 179}, { 1096, 135},
	  { 961, 105}, { 856, 85}, { 771, 69}, { 702, 58}, { 644, 49}, { 595, 42}, { 553, 37}, { 516, 32},
	  { 484, 28}, { 456, 25}, { 431, 23}, { 408, 20}, { 388, 19}, { 369, 16}, { 353, 16}, { 337, 14},
	  { 323, 13}, { 310, 11}, { 299, 11}, { 288, 11}, { 277, 9}, { 268, 9}, { 259, 8}, { 251, 8},
	  { 243, 8}, { 235, 7}, { 228, 6}, { 222, 6}, { 216, 6}, { 210, 6}, { 204, 5}, { 199, 5},
	  { 194, 5}, { 189, 4}, { 185, 4}, { 181, 4}, { 177, 4}, { 173, 4}, { 169, 4}, { 165, 3},
	  { 162, 3}, { 159, 4}, { 155, 3}, { 152, 3}, { 149, 2}, { 147, 3}, { 144, 3}, { 141, 2},
	  { 139, 3}, { 136, 2}, { 134, 2}, { 132, 3}, { 129, 2}, { 127, 2}, { 125, 2}, { 123, 2},
	  { 121, 2}, { 119, 1}, { 118, 2}, { 116, 2}, { 114, 1}, { 113, 2}, { 111, 2}, { 109, 1},
	  { 108, 2}, { 106, 1}, { 105, 2}, { 103, 1}, { 102, 1}, { 101, 1}, { 100, 2}, { 98, 1},
	  { 97, 1}, { 96, 1}, { 95, 2}, { 93, 1}, { 92, 1}, { 91, 1}, { 90, 1}, { 89, 1},
	  { 88, 1}, { 87, 1}, { 86, 1}, { 85, 1}, { 84, 1}, { 83, 0}, { 83, 1}, { 82, 1},
	  { 81, 1}, { 80, 1}, { 79, 1}, { 78, 0}, { 78, 1}, { 77, 1}, { 76, 1}, { 75, 0},
	  { 75, 1}, { 74, 1}, { 73, 1}, { 72, 0}, { 72, 1}, { 71, 1}, { 70, 0}, { 70, 1},
	  { 69, 0}, { 69, 1}, { 68, 1}, { 67, 0}, { 67, 1}, { 66, 0}, { 66, 1}, { 65, 0},
	  { 65, 1}, { 64, 1}, { 63, 0}, { 63, 1}, { 62, 0}, { 62, 1}, { 61, 0}, { 61, 1},
	  { 60, 0}, { 60, 0}, { 60, 1}, { 59, 0}, { 59, 1}, { 58, 0}, { 58, 1}, { 57, 0},
	  { 57, 1}, { 56, 0}, { 56, 0}, { 56, 1}, { 55, 0}, { 55, 1}, { 54, 0}, { 54, 0},
	  { 54, 1}, { 53, 0}, { 53, 0}, { 53, 1}, { 52, 0}, { 52, 0}, { 52, 1}, { 51, 0},
	  { 51, 0}, { 51, 1}, { 50, 0}, { 50, 0}, { 50, 1}, { 49, 0}, { 49, 0}, { 49, 1},
	  { 48, 0}, { 48, 0}, { 48, 1}, { 47, 0}, { 47, 0}, { 47, 0}, { 47, 1}, { 46, 0},
	  { 46, 0}, { 46, 1}, { 45, 0}, { 45, 0}, { 45, 0}, { 45, 1}, { 44, 0}, { 44, 0},
	  { 44, 0}, { 44, 1}, { 43, 0}, { 43, 0}, { 43, 0}, { 43, 1}, { 42, 0}, { 42, 0},
	  { 42, 0}, { 42, 1}, { 41, 0}, { 41, 0}, { 41, 0}, { 41, 0}, { 41, 1}, { 40, 0},
	  { 40, 0}, { 40, 0}, { 40, 0}, { 40, 1}, { 39, 0}, { 39, 0}, { 39, 0}, { 39, 0},
	  { 39, 1}, { 38, 0}, { 38, 0}, { 38, 0}, { 38, 0}, { 38, 1}, { 37, 0}, { 37, 0},
	  { 37, 0}, { 37, 0}, { 37, 0}, { 37, 1}, { 36, 0}, { 36, 0}, { 36, 0}, { 36, 0},
	  { 36, 1}, { 35, 0}, { 35, 0}, { 35, 0}, { 35, 0}, { 35, 0}, { 35, 0}, { 35, 1},
	  { 34, 0}, { 34, 0}, { 34, 0}, { 34, 0}, { 34, 0}, { 34, 1}, { 33, 0}, { 33, 0},
	  { 33, 0}, { 33, 0}, { 33, 0}, { 33, 0}, { 33, 1}, { 32, 0}, { 32, 0}, { 32, 0},
	  { 32, 0}, { 32, 0}, { 32, 0}, { 32, 0}, { 32, 1}, { 31, 0}, { 31, 0}, { 31, 0},
	  { 31, 0}, { 31, 0}, { 31, 0}, { 31, 1}, { 30, 0}, { 30, 0}, { 30, 0}, { 30, 0}
	};

	uint16_t const speed_lookuptable_slow[256][2] = {
	  { 62500, 12500}, { 50000, 8334}, { 41666, 5952}, { 35714, 4464}, { 31250, 3473}, { 27777, 2777}, { 25000, 2273}, { 22727, 1894},
	  { 20833, 1603}, { 19230, 1373}, { 17857, 1191}, { 16666, 1041}, { 15625, 920}, { 14705, 817}, { 13888, 731}, { 13157, 657},
	  { 12500, 596}, { 11904, 541}, { 11363, 494}, { 10869, 453}, { 10416, 416}, { 10000, 385}, { 9615, 356}, { 9259, 331},
	  { 8928, 308}, { 8620, 287}, { 8333, 269}, { 8064, 252}, { 7812, 237}, { 7575, 223}, { 7352, 210}, { 7142, 198},
	  { 6944, 188}, { 6756, 178}, { 6578, 168}, { 6410, 160}, { 6250, 153}, { 6097, 145}, { 5952, 139}, { 5813, 132},
	  { 5681, 126}, { 5555, 121}, { 5434, 115}, { 5319, 111}, { 5208, 106}, { 5102, 102}, { 5000, 99}, { 4901, 94},
	  { 4807, 91}, { 4716, 87}, { 4629, 84}, { 4545, 81}, { 4464, 79}, { 4385, 75}, { 4310, 73}, { 4237, 71},
	  { 4166, 68}, { 4098, 66}, { 4032, 64}, { 3968, 62}, { 3906, 60}, { 3846, 59}, { 3787, 56}, { 3731, 55},
	  { 3676, 53}, { 3623, 52}, { 3571, 50}, { 3521, 49}, { 3472, 48}, { 3424, 46}, { 3378, 45}, { 3333, 44},
	  { 3289, 43}, { 3246, 41}, { 3205, 41}, { 3164, 39}, { 3125, 39}, { 3086, 38}, { 3048, 36}, { 3012, 36},
	  { 2976, 35}, { 2941, 35}, { 2906, 33}, { 2873, 33}, { 2840, 32}, { 2808, 31}, { 2777, 30}, { 2747, 30},
	  { 2717, 29}, { 2688, 29}, { 2659, 28}, { 2631, 27}, { 2604, 27}, { 2577, 26}, { 2551, 26}, { 2525, 25},
	  { 2500, 25}, { 2475, 25}, { 2450, 23}, { 2427, 24}, { 2403, 23}, { 2380, 22}, { 2358, 22}, { 2336, 22},
	  { 2314, 21}, { 2293, 21}, { 2272, 20}, { 2252, 20}, { 2232, 20}, { 2212, 20}, { 2192, 19}, { 2173, 18},
	  { 2155, 19}, { 2136, 18}, { 2118, 18}, { 2100, 17}, { 2083, 17}, { 2066, 17}, { 2049, 17}, { 2032, 16},
	  { 2016, 16}, { 2000, 16}, { 1984, 16}, { 1968, 15}, { 1953, 16}, { 1937, 14}, { 1923, 15}, { 1908, 15},
	  { 1893, 14}, { 1879, 14}, { 1865, 14}, { 1851, 13}, { 1838, 14}, { 1824, 13}, { 1811, 13}, { 1798, 13},
	  { 1785, 12}, { 1773, 13}, { 1760, 12}, { 1748, 12}, { 1736, 12}, { 1724, 12}, { 1712, 12}, { 1700, 11},
	  { 1689, 12}, { 1677, 11}, { 1666, 11}, { 1655, 11}, { 1644, 11}, { 1633, 10}, { 1623, 11}, { 1612, 10},
	  { 1602, 10}, { 1592, 10}, { 1582, 10}, { 1572, 10}, { 1562, 10}, { 1552, 9}, { 1543, 10}, { 1533, 9},
	  { 1524, 9}, { 1515, 9}, { 1506, 9}, { 1497, 9}, { 1488, 9}, { 1479, 9}, { 1470, 9}, { 1461, 8},
	  { 1453, 8}, { 1445, 9}, { 1436, 8}, { 1428, 8}, { 1420, 8}, { 1412, 8}, { 1404, 8}, { 1396, 8},
	  { 1388, 7}, { 1381, 8}, { 1373, 7}, { 1366, 8}, { 1358, 7}, { 1351, 7}, { 1344, 8}, { 1336, 7},
	  { 1329, 7}, { 1322, 7}, { 1315, 7}, { 1308, 6}, { 1302, 7}, { 1295, 7}, { 1288, 6}, { 1282, 7},
	  { 1275, 6}, { 1269, 7}, { 1262, 6}, { 1256, 6}, { 1250, 7}, { 1243, 6}, { 1237, 6}, { 1231, 6},
	  { 1225, 6}, { 1219, 6}, { 1213, 6}, { 1207, 6}, { 1201, 5}, { 1196, 6}, { 1190, 6}, { 1184, 5},
	  { 1179, 6}, { 1173, 5}, { 1168, 6}, { 1162, 5}, { 1157, 5}, { 1152, 6}, { 1146, 5}, { 1141, 5},
	  { 1136, 5}, { 1131, 5}, { 1126, 5}, { 1121, 5}, { 1116, 5}, { 1111, 5}, { 1106, 5}, { 1101, 5},
	  { 1096, 5}, { 1091, 5}, { 1086, 4}, { 1082, 5}, { 1077, 5}, { 1072, 4}, { 1068, 5}, { 1063, 4},
	  { 1059, 5}, { 1054, 4}, { 1050, 4}, { 1046, 5}, { 1041, 4}, { 1037, 4}, { 1033, 5}, { 1028, 4},
	  { 1024, 4}, { 1020, 4}, { 1016, 4}, { 1012, 4}, { 1008, 4}, { 1004, 4}, { 1000, 4}, { 996, 4},
	  { 992, 4}, { 988, 4}, { 984, 4}, { 980, 4}, { 976, 4}, { 972, 4}, { 968, 3}, { 965, 3}
	};

	//
	// Private variables
	//

	TIM_HandleTypeDef htim6;

	//
	// Positions of stepper motors, in step units
	//
	volatile int32_t count_position = 0;

	uint8_t last_direction_bits = 0U;        // The next stepping-bits to be output
	uint16_t cleaning_buffer_counter = 0U;

	int32_t acceleration_time;
	int32_t deceleration_time;

	uint16_t acc_step_rate; // needed for deceleration start point
	uint8_t step_loops;
	uint8_t step_loops_nominal;
	uint16_t TIM6_ARR_nominal;

	volatile int32_t endstops_trigsteps;

	// Counter variables for the Bresenham line tracer
	int32_t counter_MOTOR = 0;
	volatile uint32_t step_events_completed = 0U; // The number of step events executed in the current block

	//
	// Current direction of stepper motors (+1 or -1)
	//
	volatile int8_t count_direction = 1;

	//
	// Public variable initialization
	//

	Planner::block_t* current_block = nullptr;  // A pointer to the block currently being traced

	//
	// Private function prototypes
	//

	//
	// Interrupt Service Routines
	//
	void isr();

	//
	// The direction of a single motor
	//
	FORCE_INLINE bool motor_direction() {
		return TEST(last_direction_bits, 0);
	}

	//
	// Set direction bits for all steppers
	//
	void set_directions();

	inline void kill_current_block() {
		step_events_completed = current_block->step_event_count;
	}

	inline uint16_t MultiU16X8toH16(uint8_t charIn1, uint32_t intIn2) {
		return (uint16_t)((intIn2 * charIn1) >> 16);
	}

	FORCE_INLINE uint16_t calc_timer(uint16_t step_rate) {
		uint16_t timer;

		step_rate = min(step_rate, MAX_STEP_FREQUENCY);

		if (step_rate > 20000) { // If steprate > 20kHz >> step 4 times
			step_rate >>= 2;
			step_loops = 4;
		} else if (step_rate > 10000) { // If steprate > 10kHz >> step 2 times
			step_rate >>= 1;
			step_loops = 2;
		} else {
			step_loops = 1;
		}

		step_rate = max(step_rate, SystemCoreClock / 500000);
		step_rate -= SystemCoreClock / 500000; // Correct for minimal speed
		if (step_rate >= (8 * 256)) { // higher step rate
			uint16_t *table_address = (uint16_t *)&speed_lookuptable_fast[(uint8_t) (step_rate >> 8)][0];
			uint8_t tmp_step_rate = (step_rate & 0x00FF);
			uint16_t gain = *(table_address + 1);
			timer = MultiU16X8toH16(tmp_step_rate, gain);
			timer = (*table_address) - timer;
		} else { // lower step rates
			uint16_t *table_address = (uint16_t *)&speed_lookuptable_slow[0][0];
			table_address += ((step_rate) >> 1) & 0xFFFFFFFC;
			timer = *table_address;
			timer -= (((*(table_address + 1)) * (unsigned char)(step_rate & 0x0007)) >> 3);
		}
		if (timer < 100) { // (20kHz - this should never happen)
			timer = 100;
		}
		return timer;
	}

	// Initialize the trapezoid generator from the current block.
	// Called whenever a new block begins.
	FORCE_INLINE void trapezoid_generator_reset() {
		if (current_block->direction_bits != last_direction_bits) {
			last_direction_bits = current_block->direction_bits;
			set_directions();
		}

		deceleration_time = 0;
		// step_rate to timer interval
		TIM6_ARR_nominal = calc_timer(current_block->nominal_rate);
		// make a note of the number of step loops required at nominal speed
		step_loops_nominal = step_loops;
		acc_step_rate = current_block->initial_rate;
		acceleration_time = calc_timer(acc_step_rate);
		TIM_SET_AUTORELOAD(&htim6, acceleration_time);
	}

	// intRes = longIn1 * longIn2 >> 24
	//
	inline int32_t MultiU24X32toH16(int32_t longIn1, int32_t longIn2) {
	    return ((int64_t)longIn1 * longIn2) >> 24;
	}

	void enableISRs(void);

	// Stepper pulse duration, in cycles
	inline uint32_t STEP_PULSE_CYCLES(void) {
		return MINIMUM_STEPPER_PULSE * Rcc::CYCLES_PER_MICROSECOND();
	}

	//
	// Namespace body
	//

	//
	// Block until all buffered steps are executed
	//
	void synchronize() {
		while (Planner::blocks_queued()) {
			Thermoprinter::idle();
		}
	}

	//
	// Set the stepper positions directly in steps
	//
	// The input is based on the typical steps.
	//
	// This allows get_position_mm to correctly
	// derive the current position later on.
	//
	void set_position(const int32_t &a) {
		synchronize(); // Bad to set stepper counts in the middle of a move

		__disable_irq();
		// default planning
		count_position = a;
		__enable_irq();
	}

	//
	// Set the stepper direction
	//
	void set_directions() {

		if (motor_direction()) {
			MOTOR_APPLY_DIR(INVERT_MOTOR_DIR, false);
			count_direction = -1;
		} else {
			MOTOR_APPLY_DIR(INVERT_MOTOR_DIR == GPIO::GPIO_PIN_RESET ? GPIO::GPIO_PIN_SET : GPIO::GPIO_PIN_RESET, false);
			count_direction = 1;
		}
	}

	void enableISRs(void) {
		__disable_irq();
		if (AdcManager::in_temp_isr) {
			NVIC_DisableIRQ(TIM2_IRQn);
		} else {
			NVIC_EnableIRQ(TIM2_IRQn);
		}
		ENABLE_STEPPER_DRIVER_INTERRUPT();
	}

	void isr() {

		uint16_t ocr_val;

		// Disable Tim6 ISRs and enable global ISR again to capture UART events (incoming chars)
		NVIC_DisableIRQ(TIM2_IRQn); \
		DISABLE_STEPPER_DRIVER_INTERRUPT();
		__enable_irq();

		if (cleaning_buffer_counter) {
			--cleaning_buffer_counter;
			current_block = nullptr;
			Planner::discard_current_block();
			TIM_SET_AUTORELOAD(&htim6, 200); // Run at max speed - 10 KHz
			enableISRs(); // re-enable ISRs
			return;
		}

		// If there is no current block, attempt to pop one from the buffer
		if (!current_block) {
			// Anything in the buffer?
			current_block = Planner::get_current_block();
			if (current_block) {
				trapezoid_generator_reset();

				// Initialize Bresenham counters to 1/2 the ceiling
				counter_MOTOR = -(current_block->step_event_count >> 1);

				step_events_completed = 0;

				Endstops::e_hit = 2; // Needed for the case an endstop is already triggered before the new move begins.
				// No 'change' can be detected.
			} else {
				TIM_SET_AUTORELOAD(&htim6, 2000); // Run at slow speed - 1 KHz
				enableISRs(); // re-enable ISRs
				return;
			}
		}

		// Update endstops state, if enabled
		if (Endstops::e_hit && Endstops::enabled) {
			Endstops::update();
			Endstops::e_hit--;
		}

		// Take multiple steps per interrupt (For high speed moves)
		bool all_steps_done = false;
		for (uint8_t i = step_loops; i--;) {

			//
			// Estimate the number of cycles that the stepper logic already takes
			// up between the start and stop of the X stepper pulse.
			//
			// Currently this uses very modest estimates of around 5 cycles.
			// True values may be derived by careful testing.
			//
			// Once any delay is added, the cost of the delay code itself
			// may be subtracted from this value to get a more accurate delay.
			// Delays under 20 cycles (1.25탎) will be very accurate, using NOPs.
			// Longer delays use a loop. The resolution is 8 cycles.
			//
			const uint32_t EXTRA_CYCLES_XYZE = STEP_PULSE_CYCLES() - 10;

			//
			// If a minimum pulse time was specified get the timer TIM2 value.
			//
			// TIM2->CNT has an 8x prescaler, so it increments every 8 cycles.
			// That's every 0.5탎 on 16MHz and every 0.4탎 on 20MHz.
			// 20 counts of TCNT0 -by itself- is a good pulse delay.
			// 10탎 = 160 or 200 cycles.
			//
			uint32_t pulse_start = 0U;
			if (EXTRA_CYCLES_XYZE > 20) {
				pulse_start = TIM_GET_COUNTER(&AdcManager::htim2);
			}

			// Advance the Bresenham counter; start a pulse if the axis needs a step
			counter_MOTOR += current_block->steps;
			if (counter_MOTOR > 0) {
				MOTOR_APPLY_STEP(INVERT_MOTOR_STEP_PIN == GPIO::GPIO_PIN_SET ? GPIO::GPIO_PIN_RESET : GPIO::GPIO_PIN_SET,0);
			}

			// For minimum pulse time wait before stopping pulses
			if (EXTRA_CYCLES_XYZE > 20) {
				while (EXTRA_CYCLES_XYZE > (uint32_t)(TIM_GET_COUNTER(&AdcManager::htim2) - pulse_start) * (8)) { }
				pulse_start = TIM_GET_COUNTER(&AdcManager::htim2);
			} else if (EXTRA_CYCLES_XYZE > 0) {
				DELAY_NOPS(EXTRA_CYCLES_XYZE);
			}

			// Stop an active pulse, reset the Bresenham counter, update the position
			if (counter_MOTOR > 0) {
			  	counter_MOTOR -= current_block->step_event_count;
			  	count_position += count_direction;
			  	MOTOR_APPLY_STEP(INVERT_MOTOR_STEP_PIN, 0);
			}

			if (++step_events_completed >= current_block->step_event_count) {
				all_steps_done = true;
				break;
			}

			// For minimum pulse time wait after stopping pulses also
			if (EXTRA_CYCLES_XYZE > 20) {
				if (i) {
					while (EXTRA_CYCLES_XYZE > (uint32_t)(TIM_GET_COUNTER(&AdcManager::htim2) - pulse_start) * (8)) { }
				}
			} else if (EXTRA_CYCLES_XYZE > 0) {
				if (i) {
					DELAY_NOPS(EXTRA_CYCLES_XYZE);
				}
			}

		} // steps_loop

		// Calculate new timer value
		if (step_events_completed <= (uint32_t)current_block->accelerate_until) {

			acc_step_rate = MultiU24X32toH16(acceleration_time, current_block->acceleration_rate);
			acc_step_rate += current_block->initial_rate;

			// upper limit
			acc_step_rate = min(acc_step_rate, current_block->nominal_rate);

			// step_rate to timer interval
			const uint16_t timer = calc_timer(acc_step_rate);

			ocr_val = timer;  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
			TIM_SET_AUTORELOAD(&htim6, ocr_val);

			acceleration_time += timer;
		} else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
			uint16_t step_rate;
			step_rate = MultiU24X32toH16(deceleration_time, current_block->acceleration_rate);

			if (step_rate < acc_step_rate) { // Still decelerating?
				step_rate = acc_step_rate - step_rate;
				step_rate = max(step_rate, current_block->final_rate);
			} else {
				step_rate = current_block->final_rate;
			}

			// step_rate to timer interval
			uint16_t const timer = calc_timer(step_rate);

			ocr_val = timer;  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
			TIM_SET_AUTORELOAD(&htim6, ocr_val);

			deceleration_time += timer;
		} else {

			ocr_val = TIM6_ARR_nominal;  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
			TIM_SET_AUTORELOAD(&htim6, ocr_val);

			// ensure we're running at the correct step rate, even if we just came off an acceleration
			step_loops = step_loops_nominal;
		}

		TIM_GET_AUTORELOAD(&htim6) = max(TIM_GET_AUTORELOAD(&htim6), TIM_GET_COUNTER(&htim6) + 16);

		// If current block is finished, reset pointer
		if (all_steps_done) {
			current_block = NULL;
			Planner::discard_current_block();
		}
		enableISRs(); // re-enable ISRs
	}

	void endstop_triggered() {

		endstops_trigsteps = count_position;

		kill_current_block();
	}

	void init() {

		// Init Dir Pin
		SET_OUTPUT(MOTOR_DIR);

		// Init Enable Pin - stepper default to disabled.
		SET_OUTPUT(MOTOR_ENABLE);
		if (!MOTOR_ENABLE_ON) {
			GPIO_WRITE_PIN(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN, GPIO::GPIO_PIN_SET);
		}

		// Init endstops and pullups
		Endstops::init();

		// Init Step Pin
		SET_OUTPUT(MOTOR_STEP);
		WRITE(MOTOR_STEP, GPIO::GPIO_PIN_RESET);
		disable_MOTOR();

		// Set the timer pre-scaler
		// Generally we use a divider of 8, resulting in a 2MHz timer
		// frequency on a 16MHz MCU. If you are going to change this, be
		// sure to regenerate speed_lookuptable.h with
		// create_speed_lookuptable.py
		TIM_Base_InitTypeDef tim6_Init;
		tim6_Init.Prescaler = 8;
		tim6_Init.CounterMode = TIM_COUNTERMODE_UP;
		// Init Stepper ISR to 122 Hz for quick starting
		tim6_Init.Period = 0x4000;
		tim6_Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

		htim6.Instance = TIM6;
		htim6.Init = tim6_Init;
		htim6.Channel = TIM_ACTIVE_CHANNEL_CLEARED;

		if (STATUS_ERROR == TIM_Base_Init(&htim6)) {
			Thermoprinter::Error_Handler(__FILE__, __LINE__);
		}

		// Set the timer pre-scaler
		// Generally we use a divider of 8, resulting in a 2MHz timer
		// frequency on a 16MHz MCU. If you are going to change this, be
		// sure to regenerate speed_lookuptable.h with
		// create_speed_lookuptable.py
		//TIM6->PSC = 8; // 1/8 prescaler

		// Init Stepper ISR to 122 Hz for quick starting
		//TIM6->ARR = 0x4000;

		// Enable update interrupts
		//TIM6->DIER |= TIM_DIER_UIE;
		TIM_ENABLE_IT(&htim6, TIM_DIER_UIE);

		// zero counter for start
		//TIM6->CNT = 0;
		TIM_RESET_COUNTER(&htim6);

		ENABLE_STEPPER_DRIVER_INTERRUPT();

		Endstops::enable(true); // Start with endstops active. After homing they can be disabled
		__enable_irq();

		set_directions(); // Init directions to last_direction_bits = 0
	}

	//
	// The stepper subsystem goes to sleep when it runs out of things to execute. Call this
	// to notify the subsystem that it is time to go to work.
	//
	void wake_up() {
		ENABLE_STEPPER_DRIVER_INTERRUPT();
	}

	void enable_MOTOR() {
		writePin(MOTOR_ENABLE_PORT, MOTOR_ENABLE_PIN, MOTOR_ENABLE_ON);
	}

	// End

}

extern "C" {
	//
	// Stepper Driver Interrupt
	//
	// Directly pulses the stepper motors at high frequency.
	// Timer TIM6 runs at a base frequency of 2MHz, with this ISR using ARR compare mode.
	//
	// ARR   Frequency
	//     1     2 MHz
	//    50    40 KHz
	//   100    20 KHz - capped max rate
	//   200    10 KHz - nominal max rate
	//  2000     1 KHz - sleep rate
	//  4000   500  Hz - init rate
	//
	void TIM6_IRQHandler(void)
	{
		Stepper::isr();
	}
}
