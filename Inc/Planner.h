#pragma once

/*
 * Planner.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#include <stdint.h>

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

	//
	// struct block_t
	//
	// A single entry in the planner buffer.
	// Tracks linear movement over multiple axes.
	//
	// The "nominal" values are as-specified by gcode, and
	// may never actually be reached due to acceleration limits.
	//
	typedef struct {

		uint8_t flag;                             // Block flags (See BlockFlag enum above)

		unsigned char active_extruder;            // The extruder to move (if E move)

		// Fields used by the Bresenham algorithm for tracing the line
		uint32_t steps;                  // Step count along each axis
		uint32_t step_event_count;                // The number of step events required to complete this block

		int32_t accelerate_until;                 // The index of the step event on which to stop acceleration
		int32_t decelerate_after;                 // The index of the step event on which to start decelerating
		int32_t acceleration_rate;                // The acceleration rate used for acceleration calculation

		uint8_t direction_bits;                   // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)

		// Fields used by the motion planner to manage acceleration
		float32_t nominal_speed;                      // The nominal speed for this block in mm/sec
		float32_t entry_speed;                        // Entry speed at previous-current junction in mm/sec
		float32_t max_entry_speed;                    // Maximum allowable junction entry speed in mm/sec
		float32_t millimeters;                        // The total travel of this block in mm
		float32_t acceleration;                       // acceleration mm/sec^2

		// Settings for the trapezoid generator
		uint32_t nominal_rate;                    // The nominal step rate for this block in step_events/sec
		uint32_t initial_rate;                    // The jerk-adjusted step rate at start of block
		uint32_t final_rate;                      // The minimal rate at exit
		uint32_t acceleration_steps_per_s2;       // acceleration steps/sec^2

		uint32_t segment_time;

	} block_t;

	#define BLOCK_BUFFER_SIZE 16U // maximize block buffer

	FORCE_INLINE uint8_t BLOCK_MOD(uint8_t n) {
		return n & (BLOCK_BUFFER_SIZE - 1U);
	}

	// Moves (or segments) with fewer steps than this will be joined with the next move
	#define MIN_STEPS_PER_SEGMENT 6U

	//
	// A ring buffer of moves described in steps
	//
	extern block_t block_buffer[BLOCK_BUFFER_SIZE];
	extern volatile uint8_t block_buffer_head;  // Index of the next block to be pushed
	extern volatile uint8_t block_buffer_tail;

	void init();

	//
	// Number of moves currently in the planner
	//
	FORCE_INLINE uint8_t movesplanned() {
		return BLOCK_MOD(block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE);
	}

	extern float32_t max_feedrate_mm_s;     // Max speeds in mm per second
	extern float32_t axis_steps_per_mm;
	extern float32_t steps_to_mm;

	extern float32_t min_feedrate_mm_s;
	extern float32_t acceleration;         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
	extern float32_t retract_acceleration; // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
	extern float32_t travel_acceleration;  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
	extern float32_t max_jerk;       // The largest speed change requiring no acceleration
	extern float32_t min_travel_feedrate_mm_s;

	extern uint32_t max_acceleration_steps_per_s2;
	extern uint32_t max_acceleration_mm_per_s2;
	extern uint32_t min_segment_time;

	//
	// Planner::_buffer_line
	//
	// Add a new direct linear movement to the buffer.
	//
	// Leveling and kinematics should be applied ahead of this.
	//
	//  a,b,c,e   - target position in mm or degrees
	//  fr_mm_s   - (target) speed of the move (mm/s)
	//  extruder  - target extruder
	//
	void buffer_line(const int32_t &m, float32_t const fr_mm_s);

	//
	// Does the buffer have any blocks queued?
	//
	FORCE_INLINE bool blocks_queued() { return (block_buffer_head != block_buffer_tail); }

	//
	// Set the planner.position and individual stepper positions.
	// Used by G92, G28, G29, and other procedures.
	//
	// Clears previous speed values.
	//
	void set_position_mm(const int32_t &m);

	void reset_acceleration_rates();
	void refresh_positioning();

	//
	// "Discards" the block and "releases" the memory.
	// Called when the current block is no longer needed.
	//
	FORCE_INLINE void discard_current_block() {
		if (blocks_queued()) {
			block_buffer_tail = BLOCK_MOD(block_buffer_tail + 1);
		}
	}

	//
	// The current block. NULL if the buffer is empty.
	// This also marks the block as busy.
	//
	block_t* get_current_block();

	//
	// The current position of the tool in absolute steps
	// Recalculated if any axis_steps_per_mm are changed by gcode
	//
	extern int32_t position;

	//
	// Get the index of the next / previous block in the ring buffer
	//
	FORCE_INLINE uint8_t next_block_index(uint8_t block_index) { return BLOCK_MOD(block_index + 1U); }
	FORCE_INLINE uint8_t prev_block_index(uint8_t block_index) { return BLOCK_MOD(block_index - 1U); }

	//
	// Speed of previous path line segment
	//
	extern float32_t previous_speed;

	//
	// Nominal speed of previous path line segment
	//
	extern float32_t previous_nominal_speed;

	//
	// Limit where 64bit math is necessary for acceleration calculation
	//
	extern uint32_t cutoff_long;

	//
	// Calculate the maximum allowable speed at this point, in order
	// to reach 'target_velocity' using 'acceleration' within a given
	// 'distance'.
	//
	float32_t max_allowable_speed(
		const float32_t &accel,
		const float32_t &target_velocity,
		const float32_t &distance
	);

	void calculate_trapezoid_for_block(
		block_t* const block,
		const float32_t &entry_factor,
		const float32_t &exit_factor
	);

	void recalculate();

	//
	// Calculate the distance (not time) it takes to accelerate
	// from initial_rate to target_rate using the given acceleration:
	//
	float32_t estimate_acceleration_distance(
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
	float32_t intersection_distance(
		const float32_t &initial_rate,
		const float32_t &final_rate,
		const float32_t &accel,
		const float32_t &distance
	);

	void reverse_pass_kernel(block_t* const current, const block_t* const next);
	void forward_pass_kernel(const block_t *previous, block_t* const current);

	void reverse_pass();
	void forward_pass();

	void recalculate_trapezoids();

	#define PLANNER_XY_FEEDRATE() (min(planner.max_feedrate_mm_s[X_AXIS], planner.max_feedrate_mm_s[Y_AXIS]))

	bool isBlockFlagSet(uint8_t const flag, BlockFlagBit const bitName);
	void setBlockFlag(uint8_t &flag, BlockFlagBit const bitName);
	void clearBlockFlag(uint8_t &flag, BlockFlagBit const bitName);

}
