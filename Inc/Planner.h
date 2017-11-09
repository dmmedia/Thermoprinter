/*
 * Planner.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef PLANNER_H_
#define PLANNER_H_

#include "macros.h"
#include "Conditionals.h"
#include "main.h"

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

enum BlockFlag {
  BLOCK_FLAG_RECALCULATE          = _BV(BLOCK_BIT_RECALCULATE),
  BLOCK_FLAG_NOMINAL_LENGTH       = _BV(BLOCK_BIT_NOMINAL_LENGTH),
  BLOCK_FLAG_START_FROM_FULL_HALT = _BV(BLOCK_BIT_START_FROM_FULL_HALT),
  BLOCK_FLAG_BUSY                 = _BV(BLOCK_BIT_BUSY)
};

/**
 * struct block_t
 *
 * A single entry in the planner buffer.
 * Tracks linear movement over multiple axes.
 *
 * The "nominal" values are as-specified by gcode, and
 * may never actually be reached due to acceleration limits.
 */
typedef struct {

  uint8_t flag;                             // Block flags (See BlockFlag enum above)

  unsigned char active_extruder;            // The extruder to move (if E move)

  // Fields used by the Bresenham algorithm for tracing the line
  int32_t steps[NUM_AXIS];                  // Step count along each axis
  uint32_t step_event_count;                // The number of step events required to complete this block

  int32_t accelerate_until,                 // The index of the step event on which to stop acceleration
          decelerate_after,                 // The index of the step event on which to start decelerating
          acceleration_rate;                // The acceleration rate used for acceleration calculation

  uint8_t direction_bits;                   // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)

  // Fields used by the motion planner to manage acceleration
  float nominal_speed,                      // The nominal speed for this block in mm/sec
        entry_speed,                        // Entry speed at previous-current junction in mm/sec
        max_entry_speed,                    // Maximum allowable junction entry speed in mm/sec
        millimeters,                        // The total travel of this block in mm
        acceleration;                       // acceleration mm/sec^2

  // Settings for the trapezoid generator
  uint32_t nominal_rate,                    // The nominal step rate for this block in step_events/sec
           initial_rate,                    // The jerk-adjusted step rate at start of block
           final_rate,                      // The minimal rate at exit
           acceleration_steps_per_s2;       // acceleration steps/sec^2

  uint32_t segment_time;

} block_t;

#define BLOCK_BUFFER_SIZE 16 // maximize block buffer

#define BLOCK_MOD(n) ((n)&(BLOCK_BUFFER_SIZE-1))

// Moves (or segments) with fewer steps than this will be joined with the next move
#define MIN_STEPS_PER_SEGMENT 6

class Planner {
public:
    /**
     * A ring buffer of moves described in steps
     */
    static block_t block_buffer[BLOCK_BUFFER_SIZE];
    static volatile uint8_t block_buffer_head,  // Index of the next block to be pushed
                            block_buffer_tail;

	Planner();
    void init();

    /**
     * Number of moves currently in the planner
     */
    static uint8_t movesplanned() { return BLOCK_MOD(block_buffer_head - block_buffer_tail + BLOCK_BUFFER_SIZE); }

    static float max_feedrate_mm_s[XYZE_N],     // Max speeds in mm per second
                 axis_steps_per_mm[XYZE_N],
                 steps_to_mm[XYZE_N];

#define ARG_X const float &lx
#define ARG_Y const float &ly
#define ARG_Z const float &lz

    static float min_feedrate_mm_s,
                 acceleration,         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
                 retract_acceleration, // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
                 travel_acceleration,  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
                 max_jerk[XYZE],       // The largest speed change requiring no acceleration
                 min_travel_feedrate_mm_s;

    static uint32_t max_acceleration_steps_per_s2[XYZE_N],
                    max_acceleration_mm_per_s2[XYZE_N]; // Use M201 to override by software
    static millis_t min_segment_time;

    /**
     * Planner::_buffer_line
     *
     * Add a new direct linear movement to the buffer.
     *
     * Leveling and kinematics should be applied ahead of this.
     *
     *  a,b,c,e   - target position in mm or degrees
     *  fr_mm_s   - (target) speed of the move (mm/s)
     *  extruder  - target extruder
     */
    static void _buffer_line(const float &a, const float &b, const float &c, const float &e, float fr_mm_s, const uint8_t extruder);

    static void _set_position_mm(const float &a, const float &b, const float &c, const float &e);

    /**
     * Add a new linear movement to the buffer.
     * The target is NOT translated to delta/scara
     *
     * Leveling will be applied to input on cartesians.
     * Kinematic machines should call buffer_line_kinematic (for leveled moves).
     * (Cartesians may also call buffer_line_kinematic.)
     *
     *  lx,ly,lz,e   - target position in mm or degrees
     *  fr_mm_s      - (target) speed of the move (mm/s)
     *  extruder     - target extruder
     */
    static FORCE_INLINE void buffer_line(ARG_X, ARG_Y, ARG_Z, const float &e, const float &fr_mm_s, const uint8_t extruder) {
      _buffer_line(lx, ly, lz, e, fr_mm_s, extruder);
    }

    /**
     * Does the buffer have any blocks queued?
     */
    static bool blocks_queued() { return (block_buffer_head != block_buffer_tail); }

    /**
     * Set the planner.position and individual stepper positions.
     * Used by G92, G28, G29, and other procedures.
     *
     * Clears previous speed values.
     */
    static FORCE_INLINE void set_position_mm(ARG_X, ARG_Y, ARG_Z, const float &e) {
      _set_position_mm(lx, ly, lz, e);
    }
    static void set_position_mm(const AxisEnum axis, const float &v);
    static FORCE_INLINE void set_e_position_mm(const float &e) { set_position_mm(AxisEnum(E_AXIS), e); }

    static void reset_acceleration_rates();
    static void refresh_positioning();

    static void set_position_mm_kinematic(const float position[NUM_AXIS]);
private:
    /**
     * The current position of the tool in absolute steps
     * Recalculated if any axis_steps_per_mm are changed by gcode
     */
    static long position[NUM_AXIS];

    /**
     * Get the index of the next / previous block in the ring buffer
     */
    static int8_t next_block_index(int8_t block_index) { return BLOCK_MOD(block_index + 1); }
    static int8_t prev_block_index(int8_t block_index) { return BLOCK_MOD(block_index - 1); }

#if ENABLED(DISABLE_INACTIVE_EXTRUDER)
  /**
   * Counters to manage disabling inactive extruders
   */
  static uint8_t g_uc_extruder_last_move[EXTRUDERS];
#endif // DISABLE_INACTIVE_EXTRUDER

  /**
   * Speed of previous path line segment
   */
  static float previous_speed[NUM_AXIS];

  /**
   * Nominal speed of previous path line segment
   */
  static float previous_nominal_speed;

  /**
   * Limit where 64bit math is necessary for acceleration calculation
   */
  static uint32_t cutoff_long;

  /**
   * Calculate the maximum allowable speed at this point, in order
   * to reach 'target_velocity' using 'acceleration' within a given
   * 'distance'.
   */
  static float max_allowable_speed(const float &accel, const float &target_velocity, const float &distance) {
    return SQRT(sq(target_velocity) - 2 * accel * distance);
  }

  static void calculate_trapezoid_for_block(block_t* const block, const float &entry_factor, const float &exit_factor);

  static void recalculate();

  /**
   * Calculate the distance (not time) it takes to accelerate
   * from initial_rate to target_rate using the given acceleration:
   */
  static float estimate_acceleration_distance(const float &initial_rate, const float &target_rate, const float &accel) {
    if (accel == 0) return 0; // accel was 0, set acceleration distance to 0
    return (sq(target_rate) - sq(initial_rate)) / (accel * 2);
  }

  /**
   * Return the point at which you must start braking (at the rate of -'acceleration') if
   * you start at 'initial_rate', accelerate (until reaching the point), and want to end at
   * 'final_rate' after traveling 'distance'.
   *
   * This is used to compute the intersection point between acceleration and deceleration
   * in cases where the "trapezoid" has no plateau (i.e., never reaches maximum speed)
   */
  static float intersection_distance(const float &initial_rate, const float &final_rate, const float &accel, const float &distance) {
    if (accel == 0) return 0; // accel was 0, set intersection distance to 0
    return (accel * 2 * distance - sq(initial_rate) + sq(final_rate)) / (accel * 4);
  }

  static void reverse_pass_kernel(block_t* const current, const block_t *next);
  static void forward_pass_kernel(const block_t *previous, block_t* const current);

  static void reverse_pass();
  static void forward_pass();

  static void recalculate_trapezoids();

};

#define PLANNER_XY_FEEDRATE() (min(planner.max_feedrate_mm_s[X_AXIS], planner.max_feedrate_mm_s[Y_AXIS]))

extern Planner planner;

#endif /* PLANNER_H_ */
