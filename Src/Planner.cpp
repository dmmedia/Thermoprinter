/*
 * Planner.cpp
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

//lint -save -e892
#include <math.h>
#include <stdint.h>
#include <stm32l0xx.h>
#include "macros.h"
#include "typedefs.h"
#include "Planner.h"
#include "SREGEmulation.h"
#include "gpio.h"
#include "Configuration.h"
#include <stdlib.h>
#include "main.h"
#include "Stepper.h"
#include "CommandProcessor.h"
#include "Thermoprinter.h"
//lint -restore

namespace Planner {
	enum BlockFlagBit {
		// Recalculate trapezoids on entry junction. For optimization.
		BLOCK_BIT_RECALCULATE,

		// Nominal speed always reached.
		// i.e., The segment is long enough, so the nominal speed is reachable if accelerating
		// from a safe speed (in consideration of jerking from zero speed).
		BLOCK_BIT_NOMINAL_LENGTH,

		// Start from a halt at the start of this block, respecting the maximum allowed jerk.
		BLOCK_BIT_START_FROM_FULL_HALT,

		// The block is busy
		BLOCK_BIT_BUSY
	};

	enum {
		BLOCK_FLAG_RECALCULATE          = (1UL << BLOCK_BIT_RECALCULATE),
		BLOCK_FLAG_NOMINAL_LENGTH       = (1UL << BLOCK_BIT_NOMINAL_LENGTH),
		BLOCK_FLAG_START_FROM_FULL_HALT = (1UL << BLOCK_BIT_START_FROM_FULL_HALT),
		BLOCK_FLAG_BUSY                 = (1UL << BLOCK_BIT_BUSY)
	};

	#define BLOCK_BUFFER_SIZE 16U // maximize block buffer

	// Moves (or segments) with fewer steps than this will be joined with the next move
	#define MIN_STEPS_PER_SEGMENT 6U

	//
	// A ring buffer of moves described in steps
	//
	static block_t block_buffer[BLOCK_BUFFER_SIZE];

	static float32_t steps_to_mm;

	static uint32_t max_acceleration_steps_per_s2;

	//
	// The current position of the tool in absolute steps
	// Recalculated if any axis_steps_per_mm are changed by gcode
	//
	static int32_t position = 0;

	//
	// Speed of previous path line segment
	//
	static float32_t previous_speed;

	//
	// Nominal speed of previous path line segment
	//
	static float32_t previous_nominal_speed;

	//
	// Limit where 64bit math is necessary for acceleration calculation
	//
	static uint32_t cutoff_long;

	static volatile uint8_t block_buffer_head = 0U;  // Index of the next block to be pushed
	static volatile uint8_t block_buffer_tail = 0U;

	//
	// Calculate the maximum allowable speed at this point, in order
	// to reach 'target_velocity' using 'acceleration' within a given
	// 'distance'.
	//
	static float32_t max_allowable_speed(
		const float32_t &accel,
		const float32_t &target_velocity,
		const float32_t &distance
	);

	static void calculate_trapezoid_for_block(
		block_t* const block,
		const float32_t &entry_factor,
		const float32_t &exit_factor
	);

	static void recalculate();

	//
	// Calculate the distance (not time) it takes to accelerate
	// from initial_rate to target_rate using the given acceleration:
	//
	static float32_t estimate_acceleration_distance(
		const float32_t &initial_rate,
		const float32_t &target_rate,
		const float32_t &accel
	);

	//
	// Return the point at which you must start braking (at the rate of -'acceleration') if
	// you start at 'initial_rate', accelerate (until reaching the point), and want to end at
	// 'final_rate' after traveling 'distance'.
	//
	// This is used to compute the intersection point between acceleration and deceleration
	// in cases where the "trapezoid" has no plateau (i.e., never reaches maximum speed)
	//
	static float32_t intersection_distance(
		const float32_t &initial_rate,
		const float32_t &final_rate,
		const float32_t &accel,
		const float32_t &distance
	);

	static void reverse_pass_kernel(block_t* const current, const block_t* const next);
	static void forward_pass_kernel(const block_t *previous, block_t* const current);

	static void reverse_pass();
	static void forward_pass();

	static void recalculate_trapezoids();

	static bool isBlockFlagSet(uint8_t const flag, BlockFlagBit const bitName);
	static void setBlockFlag(uint8_t &flag, BlockFlagBit const bitName);
	static void clearBlockFlag(uint8_t &flag, BlockFlagBit const bitName);

	FORCE_INLINE uint8_t BLOCK_MOD(uint8_t n) {
		return n & (BLOCK_BUFFER_SIZE - 1U);
	}

	//
	// Get the index of the next / previous block in the ring buffer
	//
	static FORCE_INLINE uint8_t next_block_index(uint8_t block_index) { return BLOCK_MOD(block_index + 1U); }
	static FORCE_INLINE uint8_t prev_block_index(uint8_t block_index) { return BLOCK_MOD(block_index - 1U); }

	//
	// Number of moves currently in the planner
	//
	FORCE_INLINE uint8_t movesplanned() {
		return BLOCK_MOD(block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE);
	}

	float32_t max_feedrate_mm_s; // Max speeds in mm per second
	float32_t axis_steps_per_mm;

	// Initialized by settings.load()
	float32_t min_feedrate_mm_s;
	float32_t acceleration;         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
	float32_t travel_acceleration;  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
	float32_t max_jerk;       // The largest speed change requiring no acceleration
	float32_t min_travel_feedrate_mm_s;

	uint32_t max_acceleration_mm_per_s2;

	uint32_t min_segment_time;

	//
	// Class and Instance Methods
	//

	void init() {
		block_buffer_head = 0U;
		block_buffer_tail = 0U;
		position = 0;
		previous_speed = 0.0F;
		previous_nominal_speed = 0.0F;
	}

	constexpr uint32_t MINIMAL_STEP_RATE = 120U;

	//
	// Calculate trapezoid parameters, multiplying the entry- and exit-speeds
	// by the provided factors.
	//
	static void calculate_trapezoid_for_block(
		block_t* const block,
		const float32_t &entry_factor,
		const float32_t &exit_factor
	) {
		uint32_t initial_rate = static_cast<uint32_t>(
			ceil(static_cast<float64_t>(block->nominal_rate) * entry_factor)
		);
		uint32_t final_rate = static_cast<uint32_t>(
			ceil(static_cast<float64_t>(block->nominal_rate) * exit_factor)
		); // (steps per second)

		// Limit minimal step rate (Otherwise the timer will overflow.)
		initial_rate = max(initial_rate, MINIMAL_STEP_RATE);
		final_rate = max(final_rate, MINIMAL_STEP_RATE);

		int32_t const accel = static_cast<int32_t>(block->acceleration_steps_per_s2);
		int32_t accelerate_steps = static_cast<int32_t>(ceil(
			estimate_acceleration_distance(
				static_cast<float32_t>(initial_rate),
				static_cast<float32_t>(block->nominal_rate),
				static_cast<float32_t>(accel)
			)
		));
		int32_t const decelerate_steps = static_cast<int32_t>(floor(
			estimate_acceleration_distance(
				static_cast<float32_t>(block->nominal_rate),
				static_cast<float32_t>(final_rate),
				-(static_cast<float32_t>(accel))
			)
		));
		int32_t plateau_steps = (
			static_cast<int32_t>(block->step_event_count) - accelerate_steps
		) - decelerate_steps;

		// Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
		// have to use intersection_distance() to calculate when to abort accel and start braking
		// in order to reach the final_rate exactly at the end of this block.
		if (plateau_steps < 0) {
			accelerate_steps = static_cast<int32_t>(ceil(
				intersection_distance(
					static_cast<float32_t>(initial_rate),
					static_cast<float32_t>(final_rate),
					static_cast<float32_t>(accel),
					static_cast<float32_t>(block->step_event_count)
				)
			));
			accelerate_steps = max(accelerate_steps, 0); // Check limits due to numerical round-off
			accelerate_steps = min(
				accelerate_steps,
				static_cast<int32_t>(block->step_event_count)
			);
			plateau_steps = 0;
		}

		// block->accelerate_until = accelerate_steps;
		// block->decelerate_after = accelerate_steps+plateau_steps;

		noInterrupts();  // Fill variables used by the stepper in a critical section
		if (!isBlockFlagSet(block->flag, BLOCK_BIT_BUSY)) { // Don't update variables if block is busy.
			block->accelerate_until = accelerate_steps;
			block->decelerate_after = accelerate_steps + plateau_steps;
			block->initial_rate = initial_rate;
			block->final_rate = final_rate;
		}
		interrupts();
	}

	static bool isBlockFlagSet(uint8_t const flag, BlockFlagBit const bitName) {
		uint8_t bit;
		switch (bitName) {
			case BLOCK_BIT_RECALCULATE:
				bit = 1U;
				break;
			case BLOCK_BIT_NOMINAL_LENGTH:
				bit = 2U;
				break;
			case BLOCK_BIT_START_FROM_FULL_HALT:
				bit = 4U;
				break;
			case BLOCK_BIT_BUSY:
				bit = 8U;
				break;
			default:
				bit = 0U;
				break;
		}
		return (flag & bit) != 0U;
	}
	//
	// recalculate() needs to go over the current plan twice.
	// Once in reverse and once forward. This implements the reverse pass.
	//
	static void reverse_pass() {

		if (movesplanned() > 3U) {

			block_t* block[3] = { NULL, NULL, NULL };

			// Make a local copy of block_buffer_tail, because the interrupt can alter it
			// Is a critical section REALLY needed for a single byte change?
			noInterrupts();
			uint8_t const tail = block_buffer_tail;
			interrupts();

			uint8_t b = BLOCK_MOD(block_buffer_head - 3U);
			while (b != tail) {
				if (
					(block[0U] != nullptr) &&
					isBlockFlagSet(block[0U]->flag, BLOCK_BIT_START_FROM_FULL_HALT)
				) {
					break;
				}
				b = prev_block_index(b);
				block[2U] = block[1U];
				block[1U] = block[0U];
				block[0U] = &block_buffer[b];
				reverse_pass_kernel(block[1U], block[2U]);
			}
		}
	}

	// The kernel called by recalculate() when scanning the plan from first to last entry.
	static void forward_pass_kernel(const block_t* const previous, block_t* const current) {
		if (previous != nullptr) {
			// If the previous block is an acceleration block, but it is not long enough to complete the
			// full speed change within the block, we need to adjust the entry speed accordingly. Entry
			// speeds have already been reset, maximized, and reverse planned by reverse planner.
			// If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
			if (!isBlockFlagSet(previous->flag, BLOCK_BIT_NOMINAL_LENGTH)) {
				if (previous->entry_speed < current->entry_speed) {
					float32_t const mas = max_allowable_speed(
						-previous->acceleration,
						previous->entry_speed,
						previous->millimeters
					);
					float32_t const entry_speed = min(current->entry_speed, mas);
					// Check for junction speed change
					if (fabs(current->entry_speed - entry_speed) < 0.0001) {
						current->entry_speed = entry_speed;
						setBlockFlag(current->flag, BLOCK_BIT_RECALCULATE);
					}
				}
			}
		}
	}

	static void setBlockFlag(uint8_t &flag, BlockFlagBit const bitName) {
		uint8_t bit;
		switch (bitName) {
			case BLOCK_BIT_RECALCULATE:
				bit = static_cast<uint8_t>(BLOCK_FLAG_RECALCULATE);
				break;
			case BLOCK_BIT_NOMINAL_LENGTH:
				bit = static_cast<uint8_t>(BLOCK_FLAG_NOMINAL_LENGTH);
				break;
			case BLOCK_BIT_START_FROM_FULL_HALT:
				bit = static_cast<uint8_t>(BLOCK_FLAG_START_FROM_FULL_HALT);
				break;
			case BLOCK_BIT_BUSY:
				bit = static_cast<uint8_t>(BLOCK_FLAG_BUSY);
				break;
			default:
				bit = 0U;
				break;
		}

		flag |= bit;
	}

	//
	// recalculate() needs to go over the current plan twice.
	// Once in reverse and once forward. This implements the forward pass.
	//
	static void forward_pass() {
		block_t* block[3] = { NULL, NULL, NULL };

		for (uint8_t b = block_buffer_tail; b != block_buffer_head; b = next_block_index(b)) {
			block[0U] = block[1U];
			block[1U] = block[2U];
			block[2U] = &block_buffer[b];
			forward_pass_kernel(block[0U], block[1U]);
		}
		forward_pass_kernel(block[1U], block[2U]);
	}

	//
	// Recalculate the trapezoid speed profiles for all blocks in the plan
	// according to the entry_factor for each junction. Must be called by
	// recalculate() after updating the blocks.
	//
	static void recalculate_trapezoids() {
		uint8_t block_index = block_buffer_tail;
		block_t *current;
		block_t *next = NULL;

		while (block_index != block_buffer_head) {
			current = next;
			next = &block_buffer[block_index];
			if (current != nullptr) {
				// Recalculate if current block entry or exit junction speed has changed.
				if (
					isBlockFlagSet(current->flag, BLOCK_BIT_RECALCULATE) ||
					isBlockFlagSet(next->flag, BLOCK_BIT_RECALCULATE)
				) {
					// NOTE: Entry and exit factors always > 0 by all previous logic operations.
					float32_t const nom = current->nominal_speed;
					calculate_trapezoid_for_block(
						current,
						current->entry_speed / nom,
						next->entry_speed / nom
					);
					clearBlockFlag(current->flag, BLOCK_BIT_RECALCULATE); // Reset current only to ensure next trapezoid is computed
				}
			}
			block_index = next_block_index(block_index);
		}
		// Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
		if (next != nullptr) {
			float32_t const nom = next->nominal_speed;
			calculate_trapezoid_for_block(
				next,
				next->entry_speed / nom,
				(MINIMUM_PLANNER_SPEED) / nom
			);
			clearBlockFlag(next->flag, BLOCK_BIT_RECALCULATE);
		}
	}

	static void clearBlockFlag(uint8_t &flag, BlockFlagBit const bitName) {
		uint8_t bit;
		switch (bitName) {
			case BLOCK_BIT_RECALCULATE:
				bit = 1U;
				break;
			case BLOCK_BIT_NOMINAL_LENGTH:
				bit = 2U;
				break;
			case BLOCK_BIT_START_FROM_FULL_HALT:
				bit = 4U;
				break;
			case BLOCK_BIT_BUSY:
				bit = 8U;
				break;
			default:
				bit = 0U;
				break;
		}
		flag &= ~bit;
	}
	//
	// Recalculate the motion plan according to the following algorithm:
	//
	//   1. Go over every block in reverse order...
	//
	//      Calculate a junction speed reduction (block_t.entry_factor) so:
	//
	//      a. The junction jerk is within the set limit, and
	//
	//      b. No speed reduction within one block requires faster
	//         deceleration than the one, true constant acceleration.
	//
	//   2. Go over every block in chronological order...
	//
	//      Dial down junction speed reduction values if:
	//      a. The speed increase within one block would require faster
	//         acceleration than the one, true constant acceleration.
	//
	// After that, all blocks will have an entry_factor allowing all speed changes to
	// be performed using only the one, true constant acceleration, and where no junction
	// jerk is jerkier than the set limit, Jerky. Finally it will:
	//
	//   3. Recalculate "trapezoids" for all blocks.
	//
	static void recalculate() {
		reverse_pass();
		forward_pass();
		recalculate_trapezoids();
	}

	//
	// Planner::_buffer_line
	//
	// Add a new linear movement to the buffer.
	//
	// Leveling and kinematics should be applied ahead of calling this.
	//
	//  m     - target positions in steps
	//  fr_mm_s     - (target) speed of the move
	//
	void buffer_line(const int32_t &m, float32_t const fr_mm_s) {

		// The target position of the tool in absolute steps
		// Calculate target position in absolute steps
		//this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
		const int32_t target = m;

		const int32_t dm = target - position;

		// Compute direction bit-mask for this block
		uint8_t dmask = 0U;
		if (dm < 0) {
			dmask |= 1U;
		}

		// Calculate the buffer head after we push this byte
		const uint8_t next_buffer_head = next_block_index(block_buffer_head);

		// If the buffer is full: good! That means we are well ahead of the robot.
		// Rest here until there is room in the buffer.
		while (block_buffer_tail == next_buffer_head) {
			Thermoprinter::idle();
		}

		// Prepare to set up new block
		block_t* const block = &block_buffer[block_buffer_head];

		// Clear all flags, including the "busy" bit
		block->flag = 0U;

		// Set direction bits
		block->direction_bits = dmask;

		// Number of steps for each axis
		// default non-h-bot planning
		block->steps = static_cast<uint32_t>(abs(dm));

		block->step_event_count = static_cast<uint32_t>(block->steps);

		// Bail if this is a zero-length block
		if (block->step_event_count >= MIN_STEPS_PER_SEGMENT) {

			block->active_extruder = 0U;

			//enable active axes
			if (block->steps != 0U) {
				Stepper::enable_MOTOR();
			}

			float32_t const fr_mm_s_limited = max(fr_mm_s, min_travel_feedrate_mm_s);

			//
			// This part of the code calculates the total length of the movement.
			//
			float32_t const delta_mm = static_cast<float32_t>(dm) * steps_to_mm;

			if (block->steps < MIN_STEPS_PER_SEGMENT) {
				block->millimeters = fabs(delta_mm);
			} else {
				block->millimeters = delta_mm;
			}
			float32_t const inverse_millimeters = 1.0F / block->millimeters;  // Inverse millimeters to remove multiple divides

			// Calculate moves/second for this move. No divide by zero due to previous checks.
			float32_t inverse_mm_s = fr_mm_s_limited * inverse_millimeters;

			const uint8_t moves_queued = movesplanned();

			// Slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
			// Segment time im micro seconds
			uint32_t const segment_time = static_cast<uint32_t>(lround(1000000.0 / inverse_mm_s));
			if (WITHIN(moves_queued, 2U, ((BLOCK_BUFFER_SIZE) / 2U) - 1U)) {
				if (segment_time < min_segment_time) {
					// buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
					uint32_t const segment_delta_time = min_segment_time - segment_time;
					float32_t const ddtime_per_move = (
						2.0F * static_cast<float32_t>(segment_delta_time)
					) /
						static_cast<float32_t>(moves_queued);
					uint32_t const increased_segment_time = segment_time +
						static_cast<uint32_t>(lround(ddtime_per_move));
					inverse_mm_s = 1000000.0F / static_cast<float32_t>(increased_segment_time);
				}
			}

			block->nominal_speed = block->millimeters * inverse_mm_s; // (mm/sec) Always > 0
			block->nominal_rate = static_cast<uint32_t>(ceil(
				static_cast<float32_t>(block->step_event_count) * inverse_mm_s
			)); // (step/sec) Always > 0

			// Calculate and limit speed in mm/sec for each axis
			float32_t current_speed;
			float32_t speed_factor = 1.0F; // factor <1 decreases speed
			current_speed = delta_mm * inverse_mm_s;
			const float32_t cs = fabs(current_speed);
			if (cs > max_feedrate_mm_s) {
				speed_factor = min(speed_factor, max_feedrate_mm_s / cs);
			}

			// Correct the speed
			if (speed_factor < 1.0F) {
				current_speed *= speed_factor;
				block->nominal_speed *= speed_factor;
				float32_t const nom = static_cast<float32_t>(block->nominal_rate) * speed_factor;
				block->nominal_rate = static_cast<uint32_t>(nom);
			}

			// Compute and limit the acceleration rate for the trapezoid generator.
			const float32_t steps_per_mm = static_cast<float32_t>(block->step_event_count) *
				inverse_millimeters;

			// Start with print or travel acceleration
			uint32_t accel = static_cast<uint32_t>(ceil(travel_acceleration * steps_per_mm));

			if (block->steps > 0U) {
				// Limit acceleration per axis
				if (block->step_event_count <= cutoff_long) {
					if (max_acceleration_steps_per_s2 < accel) {
						uint32_t const comp = max_acceleration_steps_per_s2 *
							block->step_event_count;
						if ((accel * block->steps) > comp) {
							accel = comp / block->steps;
						}
					}
				} else {
					if (max_acceleration_steps_per_s2 < accel) {
						uint32_t const comp = max_acceleration_steps_per_s2 *
							block->step_event_count;
						if (
							(static_cast<float32_t>(accel) *
								static_cast<float32_t>(block->steps)
							) > static_cast<float32_t>(comp)
						) {
							float32_t const faccel = static_cast<float32_t>(comp) /
									static_cast<float32_t>(block->steps);
							accel = static_cast<uint32_t>(faccel);
						}
					}
				}
			}
			block->acceleration_steps_per_s2 = accel;
			block->acceleration = static_cast<float32_t>(accel) / steps_per_mm;
			float32_t const faccel = static_cast<float32_t>(accel) * 16777216.0F;
			float32_t const fscc = static_cast<float32_t>(SystemCoreClock) * 0.125F;
			float32_t const faccel_per_tick = faccel / fscc;
			block->acceleration_rate = static_cast<int32_t>(faccel_per_tick);

			// Initial limit on the segment entry velocity
			float32_t vmax_junction;

			//
			// Adapted from Průša MKS firmware
			// https://github.com/prusa3d/Prusa-Firmware
			//
			// Start with a safe speed (from which the machine may halt to stop immediately).
			//

			// Exit speed limited by a jerk to full halt of a previous last segment
			static float32_t previous_safe_speed = 0.F;

			float32_t safe_speed = block->nominal_speed;
			uint8_t limited = 0U;
			float32_t jerk = fabs(current_speed);
			const float32_t maxj = max_jerk;
			if (jerk > maxj) {
				++limited;
				safe_speed = maxj;
			}

			if ((moves_queued > 1U) && (previous_nominal_speed > 0.0001F)) {
				// Estimate a maximum velocity allowed at a joint of two successive segments.
				// If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
				// then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

				// The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
				bool const prev_speed_larger = previous_nominal_speed > block->nominal_speed;
				float32_t const smaller_speed_factor = prev_speed_larger ?
					(block->nominal_speed / previous_nominal_speed) :
					(previous_nominal_speed / block->nominal_speed);
				// Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
				vmax_junction = prev_speed_larger ?
					block->nominal_speed :
					previous_nominal_speed;
				// Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
				float32_t v_factor = 1.0F;
				limited = 0U;
				// Now limit the jerk in all axes.
				// Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
				float32_t v_exit = previous_speed;
				float32_t const v_entry = current_speed;
				if (prev_speed_larger) {
					v_exit *= smaller_speed_factor;
				}

				// Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
				jerk = (v_exit > v_entry) ? //                                  coasting             axis reversal
					(
						((v_entry > 0.F) || (v_exit < 0.F)) ?
							(v_exit - v_entry) :
							max(v_exit, -v_entry)
					) : // v_exit <= v_entry                coasting             axis reversal
					(
						((v_entry < 0.F) || (v_exit > 0.F)) ?
							(v_entry - v_exit) :
							max(-v_exit, v_entry)
					);

				if (jerk > max_jerk) {
					v_factor *= max_jerk / jerk;
					++limited;
				}
				if (limited != 0U) {
					vmax_junction *= v_factor;
				}
				// Now the transition velocity is known, which maximizes the shared exit / entry velocity while
				// respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
				const float32_t vmax_junction_threshold = vmax_junction * 0.99F;
				if (
					(previous_safe_speed > vmax_junction_threshold) &&
					(safe_speed > vmax_junction_threshold)) {
					// Not coasting. The machine will stop and start the movements anyway,
					// better to start the segment from start.
					setBlockFlag(block->flag, BLOCK_BIT_START_FROM_FULL_HALT);
					vmax_junction = safe_speed;
				}
			} else {
				setBlockFlag(block->flag, BLOCK_BIT_START_FROM_FULL_HALT);
				vmax_junction = safe_speed;
			}

			// Max entry speed of this block equals the max exit speed of the previous block.
			block->max_entry_speed = vmax_junction;

			// Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
			const float32_t v_allowable = max_allowable_speed(
				-block->acceleration,
				MINIMUM_PLANNER_SPEED,
				block->millimeters
			);
			block->entry_speed = min(vmax_junction, v_allowable);

			// Initialize planner efficiency flags
			// Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
			// If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
			// the current block and next block junction speeds are guaranteed to always be at their maximum
			// junction speeds in deceleration and acceleration, respectively. This is due to how the current
			// block nominal speed limits both the current and next maximum junction speeds. Hence, in both
			// the reverse and forward planners, the corresponding block junction speed will always be at the
			// the maximum junction speed and may always be ignored for any speed reduction checks.
			setBlockFlag(block->flag, BLOCK_BIT_RECALCULATE);
			if (block->nominal_speed <= v_allowable) {
				setBlockFlag(block->flag, BLOCK_BIT_NOMINAL_LENGTH);
			}

			// Update previous path unit_vector and nominal speed
			previous_speed = current_speed;
			previous_nominal_speed = block->nominal_speed;
			previous_safe_speed = safe_speed;

			calculate_trapezoid_for_block(
				block,
				block->entry_speed / block->nominal_speed,
				safe_speed / block->nominal_speed
			);

			// Move buffer head
			block_buffer_head = next_buffer_head;

			// Update the position (only when a move was queued)
			position = target;

			recalculate();

			Stepper::wake_up();
		}

	} // buffer_line()

	//
	// Directly set the planner XYZ position (and stepper positions)
	// converting mm (or angles for SCARA) into steps.
	//
	// On CORE machines stepper ABC will be translated from the given XYZ.
	//

	void set_position_mm(const int32_t &m) {
		int32_t const nm = lround(static_cast<float32_t>(m) * axis_steps_per_mm);
		position = nm;
		Stepper::set_position(nm);
		previous_nominal_speed = 0.0F; // Resets planner junction speeds. Assumes start from rest.
		previous_speed = 0.0F;
	}

	// Recalculate the steps/s^2 acceleration rates, based on the mm/s^2
	void reset_acceleration_rates() {
		uint32_t highest_rate = 1U;
		float32_t const fmasps2 = static_cast<float32_t>(max_acceleration_mm_per_s2) *
			axis_steps_per_mm;
		max_acceleration_steps_per_s2 = static_cast<uint32_t>(fmasps2);
		highest_rate = max(highest_rate, max_acceleration_steps_per_s2);
		cutoff_long = 4294967295UL / highest_rate;
	}

	// Recalculate position, steps_to_mm if axis_steps_per_mm changes!
	void refresh_positioning() {
		steps_to_mm = 1.0F / axis_steps_per_mm;
		set_position_mm(static_cast<int32_t>(CommandProcessor::current_position));
		reset_acceleration_rates();
	}

	static float32_t intersection_distance(
		const float32_t &initial_rate,
		const float32_t &final_rate,
		const float32_t &accel,
		const float32_t &distance
	) {
		float32_t id = 0.F;
		if (fabs(0.F - accel) > 0.00001F) {
			id = ((((accel * 2.F) * distance) - sq(initial_rate)) + sq(final_rate)) / (accel * 4.F);
		}
		return id;
	}

	static float32_t estimate_acceleration_distance(
		const float32_t &initial_rate,
		const float32_t &target_rate,
		const float32_t &accel
	) {
		float32_t ead = 0.F;
		if (fabs(0.F - accel) > 0.00001F) {
			ead = (sq(target_rate) - sq(initial_rate)) / (accel * 2.F);
		}
		return ead;
	}

	static float32_t max_allowable_speed(
		const float32_t &accel,
		const float32_t &target_velocity,
		const float32_t &distance
	) {
		return sqrt(sq(target_velocity) - ((2.F * accel) * distance));
	}

	block_t* get_current_block() {
		block_t* curBlock = nullptr;
		if (blocks_queued()) {
			curBlock = &block_buffer[block_buffer_tail];
			setBlockFlag(curBlock->flag, BLOCK_BIT_BUSY);
		}
		return curBlock;
	}

	// The kernel called by recalculate() when scanning the plan from last to first entry.
	static void reverse_pass_kernel(block_t* const current, const block_t* const next) {
		if ((current != nullptr) && (next != nullptr)) {
			// If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
			// If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
			// check for maximum allowable speed reductions to ensure maximum possible planned speed.
			float32_t const max_entry_speed = current->max_entry_speed;
			if (fabs(current->entry_speed - max_entry_speed) > 0.00001F) {
				// If nominal length true, max junction speed is guaranteed to be reached. Only compute
				// for max allowable speed if block is decelerating and nominal length is false.
				float32_t const mas = max_allowable_speed(
					-current->acceleration,
					next->entry_speed,
					current->millimeters
				);
				current->entry_speed = (
					isBlockFlagSet(current->flag, BLOCK_BIT_NOMINAL_LENGTH) ||
					(max_entry_speed < next->entry_speed)
				) ?
					max_entry_speed :
					min(max_entry_speed, mas);
				setBlockFlag(current->flag, BLOCK_BIT_RECALCULATE);
			}
		}
	}

	//
	// "Discards" the block and "releases" the memory.
	// Called when the current block is no longer needed.
	//
	void discard_current_block() {
		if (blocks_queued()) {
			block_buffer_tail = BLOCK_MOD(block_buffer_tail + 1);
		}
	}

	//
	// Does the buffer have any blocks queued?
	//
	bool blocks_queued() {
		return (block_buffer_head != block_buffer_tail);
	}

}
