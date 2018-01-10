#pragma once

/*
 * Planner.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#include <stdint.h>

namespace Planner {
	//
	// Public definitions and constants
	//

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

	//
	// Public variables
	//

	extern float32_t max_feedrate_mm_s;     // Max speeds in mm per second
	extern float32_t axis_steps_per_mm;

	extern float32_t min_feedrate_mm_s;
	extern float32_t acceleration;         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
	extern float32_t retract_acceleration; // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
	extern float32_t travel_acceleration;  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
	extern float32_t max_jerk;       // The largest speed change requiring no acceleration
	extern float32_t min_travel_feedrate_mm_s;

	extern uint32_t max_acceleration_mm_per_s2;
	extern uint32_t min_segment_time;

	//
	// Public functions
	//

	void init();

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
	// Set the planner.position and individual stepper positions.
	// Used by G92, G28, G29, and other procedures.
	//
	// Clears previous speed values.
	//
	void set_position_mm(const int32_t &m);

	void reset_acceleration_rates();

	void refresh_positioning();

	//
	// The current block. NULL if the buffer is empty.
	// This also marks the block as busy.
	//
	block_t* get_current_block();

	//
	// "Discards" the block and "releases" the memory.
	// Called when the current block is no longer needed.
	//
	void discard_current_block();

	//
	// Does the buffer have any blocks queued?
	//
	bool blocks_queued();

	// End

}
