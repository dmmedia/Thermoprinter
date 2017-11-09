/*
 * Planner.cpp
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#include "Planner.h"
#include "main.h"

float Planner::max_feedrate_mm_s[XYZE_N], // Max speeds in mm per second
      Planner::axis_steps_per_mm[XYZE_N],
      Planner::steps_to_mm[XYZE_N];

long Planner::position[NUM_AXIS] = { 0 };

uint32_t Planner::cutoff_long;

float Planner::previous_speed[NUM_AXIS],
      Planner::previous_nominal_speed;

#if ENABLED(DISABLE_INACTIVE_EXTRUDER)
  uint8_t Planner::g_uc_extruder_last_move[EXTRUDERS] = { 0 };
#endif

Planner planner;

/**
 * A ring buffer of moves described in steps
 */
block_t Planner::block_buffer[BLOCK_BUFFER_SIZE];
volatile uint8_t Planner::block_buffer_head = 0,           // Index of the next block to be pushed
                 Planner::block_buffer_tail = 0;

// Initialized by settings.load()
float Planner::min_feedrate_mm_s,
      Planner::acceleration,         // Normal acceleration mm/s^2  DEFAULT ACCELERATION for all printing moves. M204 SXXXX
      Planner::retract_acceleration, // Retract acceleration mm/s^2 filament pull-back and push-forward while standing still in the other axes M204 TXXXX
      Planner::travel_acceleration,  // Travel acceleration mm/s^2  DEFAULT ACCELERATION for all NON printing moves. M204 MXXXX
      Planner::max_jerk[XYZE],       // The largest speed change requiring no acceleration
      Planner::min_travel_feedrate_mm_s;

uint32_t Planner::max_acceleration_steps_per_s2[XYZE_N],
         Planner::max_acceleration_mm_per_s2[XYZE_N]; // Use M201 to override by software

millis_t Planner::min_segment_time;

/**
 * Class and Instance Methods
 */

Planner::Planner() { init(); }

void Planner::init() {
  block_buffer_head = block_buffer_tail = 0;
  ZERO(position);
  ZERO(previous_speed);
  previous_nominal_speed = 0.0;
}

#define MINIMAL_STEP_RATE 120

/**
 * Calculate trapezoid parameters, multiplying the entry- and exit-speeds
 * by the provided factors.
 */
void Planner::calculate_trapezoid_for_block(block_t* const block, const float &entry_factor, const float &exit_factor) {
  uint32_t initial_rate = CEIL(block->nominal_rate * entry_factor),
           final_rate = CEIL(block->nominal_rate * exit_factor); // (steps per second)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  NOLESS(initial_rate, MINIMAL_STEP_RATE);
  NOLESS(final_rate, MINIMAL_STEP_RATE);

  int32_t accel = block->acceleration_steps_per_s2,
          accelerate_steps = CEIL(estimate_acceleration_distance(initial_rate, block->nominal_rate, accel)),
          decelerate_steps = FLOOR(estimate_acceleration_distance(block->nominal_rate, final_rate, -accel)),
          plateau_steps = block->step_event_count - accelerate_steps - decelerate_steps;

  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort accel and start braking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = CEIL(intersection_distance(initial_rate, final_rate, accel, block->step_event_count));
    NOLESS(accelerate_steps, 0); // Check limits due to numerical round-off
    accelerate_steps = min((uint32_t)accelerate_steps, block->step_event_count);//(We can cast here to unsigned, because the above line ensures that we are above zero)
    plateau_steps = 0;
  }

  // block->accelerate_until = accelerate_steps;
  // block->decelerate_after = accelerate_steps+plateau_steps;

  CRITICAL_SECTION_START;  // Fill variables used by the stepper in a critical section
  if (!TEST(block->flag, BLOCK_BIT_BUSY)) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps + plateau_steps;
    block->initial_rate = initial_rate;
    block->final_rate = final_rate;
  }
  CRITICAL_SECTION_END;
}

/**
 * recalculate() needs to go over the current plan twice.
 * Once in reverse and once forward. This implements the reverse pass.
 */
void Planner::reverse_pass() {

  if (movesplanned() > 3) {

    block_t* block[3] = { NULL, NULL, NULL };

    // Make a local copy of block_buffer_tail, because the interrupt can alter it
    // Is a critical section REALLY needed for a single byte change?
    //CRITICAL_SECTION_START;
    uint8_t tail = block_buffer_tail;
    //CRITICAL_SECTION_END

    uint8_t b = BLOCK_MOD(block_buffer_head - 3);
    while (b != tail) {
      if (block[0] && TEST(block[0]->flag, BLOCK_BIT_START_FROM_FULL_HALT)) break;
      b = prev_block_index(b);
      block[2] = block[1];
      block[1] = block[0];
      block[0] = &block_buffer[b];
      reverse_pass_kernel(block[1], block[2]);
    }
  }
}

// The kernel called by recalculate() when scanning the plan from first to last entry.
void Planner::forward_pass_kernel(const block_t* previous, block_t* const current) {
  if (!previous) return;

  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!TEST(previous->flag, BLOCK_BIT_NOMINAL_LENGTH)) {
    if (previous->entry_speed < current->entry_speed) {
      float entry_speed = min(current->entry_speed,
                               max_allowable_speed(-previous->acceleration, previous->entry_speed, previous->millimeters));
      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        SBI(current->flag, BLOCK_BIT_RECALCULATE);
      }
    }
  }
}

/**
 * recalculate() needs to go over the current plan twice.
 * Once in reverse and once forward. This implements the forward pass.
 */
void Planner::forward_pass() {
  block_t* block[3] = { NULL, NULL, NULL };

  for (uint8_t b = block_buffer_tail; b != block_buffer_head; b = next_block_index(b)) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[b];
    forward_pass_kernel(block[0], block[1]);
  }
  forward_pass_kernel(block[1], block[2]);
}

/**
 * Recalculate the trapezoid speed profiles for all blocks in the plan
 * according to the entry_factor for each junction. Must be called by
 * recalculate() after updating the blocks.
 */
void Planner::recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current, *next = NULL;

  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (TEST(current->flag, BLOCK_BIT_RECALCULATE) || TEST(next->flag, BLOCK_BIT_RECALCULATE)) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        float nom = current->nominal_speed;
        calculate_trapezoid_for_block(current, current->entry_speed / nom, next->entry_speed / nom);
        CBI(current->flag, BLOCK_BIT_RECALCULATE); // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index(block_index);
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if (next) {
    float nom = next->nominal_speed;
    calculate_trapezoid_for_block(next, next->entry_speed / nom, (MINIMUM_PLANNER_SPEED) / nom);
    CBI(next->flag, BLOCK_BIT_RECALCULATE);
  }
}

/*
 * Recalculate the motion plan according to the following algorithm:
 *
 *   1. Go over every block in reverse order...
 *
 *      Calculate a junction speed reduction (block_t.entry_factor) so:
 *
 *      a. The junction jerk is within the set limit, and
 *
 *      b. No speed reduction within one block requires faster
 *         deceleration than the one, true constant acceleration.
 *
 *   2. Go over every block in chronological order...
 *
 *      Dial down junction speed reduction values if:
 *      a. The speed increase within one block would require faster
 *         acceleration than the one, true constant acceleration.
 *
 * After that, all blocks will have an entry_factor allowing all speed changes to
 * be performed using only the one, true constant acceleration, and where no junction
 * jerk is jerkier than the set limit, Jerky. Finally it will:
 *
 *   3. Recalculate "trapezoids" for all blocks.
 */
void Planner::recalculate() {
  reverse_pass();
  forward_pass();
  recalculate_trapezoids();
}

/**
 * Planner::_buffer_line
 *
 * Add a new linear movement to the buffer.
 *
 * Leveling and kinematics should be applied ahead of calling this.
 *
 *  a,b,c,e     - target positions in mm or degrees
 *  fr_mm_s     - (target) speed of the move
 *  extruder    - target extruder
 */
void Planner::_buffer_line(const float &a, const float &b, const float &c, const float &e, float fr_mm_s, const uint8_t extruder) {

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  const long target[XYZE] = {
    LROUND(a * axis_steps_per_mm[X_AXIS]),
    LROUND(b * axis_steps_per_mm[Y_AXIS]),
    LROUND(c * axis_steps_per_mm[Z_AXIS]),
    LROUND(e * axis_steps_per_mm[E_AXIS_N])
  };

  const long da = target[X_AXIS] - position[X_AXIS],
             db = target[Y_AXIS] - position[Y_AXIS],
             dc = target[Z_AXIS] - position[Z_AXIS];

  long de = target[E_AXIS] - position[E_AXIS];

  // Compute direction bit-mask for this block
  uint8_t dm = 0;
  if (da < 0) SBI(dm, X_AXIS);
  if (db < 0) SBI(dm, Y_AXIS);
  if (dc < 0) SBI(dm, Z_AXIS);
  if (de < 0) SBI(dm, E_AXIS);

  const float esteps_float = de * volumetric_multiplier[extruder] * flow_percentage[extruder] * 0.01;
  const int32_t esteps = abs(esteps_float) + 0.5;

  // Calculate the buffer head after we push this byte
  const uint8_t next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot.
  // Rest here until there is room in the buffer.
  while (block_buffer_tail == next_buffer_head) idle();

  // Prepare to set up new block
  block_t* block = &block_buffer[block_buffer_head];

  // Clear all flags, including the "busy" bit
  block->flag = 0;

  // Set direction bits
  block->direction_bits = dm;

  // Number of steps for each axis
  // default non-h-bot planning
  block->steps[X_AXIS] = labs(da);
  block->steps[Y_AXIS] = labs(db);
  block->steps[Z_AXIS] = labs(dc);

  block->steps[E_AXIS] = esteps;
  block->step_event_count = MAX4(block->steps[X_AXIS], block->steps[Y_AXIS], block->steps[Z_AXIS], esteps);

  // Bail if this is a zero-length block
  if (block->step_event_count < MIN_STEPS_PER_SEGMENT) return;

  block->active_extruder = extruder;

  //enable active axes
  if (block->steps[X_AXIS]) enable_X();
  if (block->steps[Y_AXIS]) enable_Y();
  if (block->steps[Z_AXIS]) enable_Z();

  // Enable extruder(s)
  if (esteps) {

      #define DISABLE_IDLE_E(N) if (!g_uc_extruder_last_move[N]) disable_E##N();

      for (uint8_t i = 0; i < EXTRUDERS; i++)
        if (g_uc_extruder_last_move[i] > 0) g_uc_extruder_last_move[i]--;

      switch(extruder) {
        case 0:
          enable_E0();
          g_uc_extruder_last_move[0] = (BLOCK_BUFFER_SIZE) * 2;
        break;
      }
  }

  if (esteps)
    NOLESS(fr_mm_s, min_feedrate_mm_s);
  else
    NOLESS(fr_mm_s, min_travel_feedrate_mm_s);

  /**
   * This part of the code calculates the total length of the movement.
   * For cartesian bots, the X_AXIS is the real X movement and same for Y_AXIS.
   * But for corexy bots, that is not true. The "X_AXIS" and "Y_AXIS" motors (that should be named to A_AXIS
   * and B_AXIS) cannot be used for X and Y length, because A=X+Y and B=X-Y.
   * So we need to create other 2 "AXIS", named X_HEAD and Y_HEAD, meaning the real displacement of the Head.
   * Having the real displacement of the head, we can calculate the total movement length and apply the desired speed.
   */
  float delta_mm[XYZE];
  delta_mm[X_AXIS] = da * steps_to_mm[X_AXIS];
  delta_mm[Y_AXIS] = db * steps_to_mm[Y_AXIS];
  delta_mm[Z_AXIS] = dc * steps_to_mm[Z_AXIS];
  delta_mm[E_AXIS] = esteps_float * steps_to_mm[E_AXIS_N];

  if (block->steps[X_AXIS] < MIN_STEPS_PER_SEGMENT && block->steps[Y_AXIS] < MIN_STEPS_PER_SEGMENT && block->steps[Z_AXIS] < MIN_STEPS_PER_SEGMENT) {
    block->millimeters = FABS(delta_mm[E_AXIS]);
  }
  else {
    block->millimeters = SQRT(
      sq(delta_mm[X_AXIS]) + sq(delta_mm[Y_AXIS]) + sq(delta_mm[Z_AXIS])
    );
  }
  float inverse_millimeters = 1.0 / block->millimeters;  // Inverse millimeters to remove multiple divides

  // Calculate moves/second for this move. No divide by zero due to previous checks.
  float inverse_mm_s = fr_mm_s * inverse_millimeters;

  const uint8_t moves_queued = movesplanned();

  // Slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
  // Segment time im micro seconds
  unsigned long segment_time = LROUND(1000000.0 / inverse_mm_s);
  if (WITHIN(moves_queued, 2, (BLOCK_BUFFER_SIZE) / 2 - 1)) {
    if (segment_time < min_segment_time) {
      // buffer is draining, add extra time.  The amount of time added increases if the buffer is still emptied more.
      inverse_mm_s = 1000000.0 / (segment_time + LROUND(2 * (min_segment_time - segment_time) / moves_queued));
    }
  }

  block->nominal_speed = block->millimeters * inverse_mm_s; // (mm/sec) Always > 0
  block->nominal_rate = CEIL(block->step_event_count * inverse_mm_s); // (step/sec) Always > 0

  // Calculate and limit speed in mm/sec for each axis
  float current_speed[NUM_AXIS], speed_factor = 1.0; // factor <1 decreases speed
  LOOP_XYZE(i) {
    const float cs = FABS(current_speed[i] = delta_mm[i] * inverse_mm_s);
    if (cs > max_feedrate_mm_s[i]) NOMORE(speed_factor, max_feedrate_mm_s[i] / cs);
  }

  // Correct the speed
  if (speed_factor < 1.0) {
    LOOP_XYZE(i) current_speed[i] *= speed_factor;
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.
  const float steps_per_mm = block->step_event_count * inverse_millimeters;
  uint32_t accel;
  if (!block->steps[X_AXIS] && !block->steps[Y_AXIS] && !block->steps[Z_AXIS]) {
    // convert to: acceleration steps/sec^2
    accel = CEIL(retract_acceleration * steps_per_mm);
  }
  else {
    #define LIMIT_ACCEL_LONG(AXIS,INDX) do{ \
      if (block->steps[AXIS] && max_acceleration_steps_per_s2[AXIS+INDX] < accel) { \
        const uint32_t comp = max_acceleration_steps_per_s2[AXIS+INDX] * block->step_event_count; \
        if (accel * block->steps[AXIS] > comp) accel = comp / block->steps[AXIS]; \
      } \
    }while(0)

    #define LIMIT_ACCEL_FLOAT(AXIS,INDX) do{ \
      if (block->steps[AXIS] && max_acceleration_steps_per_s2[AXIS+INDX] < accel) { \
        const float comp = (float)max_acceleration_steps_per_s2[AXIS+INDX] * (float)block->step_event_count; \
        if ((float)accel * (float)block->steps[AXIS] > comp) accel = comp / (float)block->steps[AXIS]; \
      } \
    }while(0)

    // Start with print or travel acceleration
    accel = CEIL((esteps ? acceleration : travel_acceleration) * steps_per_mm);

    #define ACCEL_IDX 0

    // Limit acceleration per axis
    if (block->step_event_count <= cutoff_long) {
      LIMIT_ACCEL_LONG(X_AXIS, 0);
      LIMIT_ACCEL_LONG(Y_AXIS, 0);
      LIMIT_ACCEL_LONG(Z_AXIS, 0);
      LIMIT_ACCEL_LONG(E_AXIS, ACCEL_IDX);
    }
    else {
      LIMIT_ACCEL_FLOAT(X_AXIS, 0);
      LIMIT_ACCEL_FLOAT(Y_AXIS, 0);
      LIMIT_ACCEL_FLOAT(Z_AXIS, 0);
      LIMIT_ACCEL_FLOAT(E_AXIS, ACCEL_IDX);
    }
  }
  block->acceleration_steps_per_s2 = accel;
  block->acceleration = accel / steps_per_mm;
  block->acceleration_rate = (long)(accel * 16777216.0 / (HAL_RCC_GetHCLKFreq() * 0.125)); // * 8.388608

  // Initial limit on the segment entry velocity
  float vmax_junction;

  /**
   * Adapted from Průša MKS firmware
   * https://github.com/prusa3d/Prusa-Firmware
   *
   * Start with a safe speed (from which the machine may halt to stop immediately).
   */

  // Exit speed limited by a jerk to full halt of a previous last segment
  static float previous_safe_speed;

  float safe_speed = block->nominal_speed;
  uint8_t limited = 0;
  LOOP_XYZE(i) {
    const float jerk = FABS(current_speed[i]), maxj = max_jerk[i];
    if (jerk > maxj) {
      if (limited) {
        const float mjerk = maxj * block->nominal_speed;
        if (jerk * safe_speed > mjerk) safe_speed = mjerk / jerk;
      }
      else {
        ++limited;
        safe_speed = maxj;
      }
    }
  }

  if (moves_queued > 1 && previous_nominal_speed > 0.0001) {
    // Estimate a maximum velocity allowed at a joint of two successive segments.
    // If this maximum velocity allowed is lower than the minimum of the entry / exit safe velocities,
    // then the machine is not coasting anymore and the safe entry / exit velocities shall be used.

    // The junction velocity will be shared between successive segments. Limit the junction velocity to their minimum.
    bool prev_speed_larger = previous_nominal_speed > block->nominal_speed;
    float smaller_speed_factor = prev_speed_larger ? (block->nominal_speed / previous_nominal_speed) : (previous_nominal_speed / block->nominal_speed);
    // Pick the smaller of the nominal speeds. Higher speed shall not be achieved at the junction during coasting.
    vmax_junction = prev_speed_larger ? block->nominal_speed : previous_nominal_speed;
    // Factor to multiply the previous / current nominal velocities to get componentwise limited velocities.
    float v_factor = 1.f;
    limited = 0;
    // Now limit the jerk in all axes.
    LOOP_XYZE(axis) {
      // Limit an axis. We have to differentiate: coasting, reversal of an axis, full stop.
      float v_exit = previous_speed[axis], v_entry = current_speed[axis];
      if (prev_speed_larger) v_exit *= smaller_speed_factor;
      if (limited) {
        v_exit *= v_factor;
        v_entry *= v_factor;
      }

      // Calculate jerk depending on whether the axis is coasting in the same direction or reversing.
      const float jerk = (v_exit > v_entry)
          ? //                                  coasting             axis reversal
            ( (v_entry > 0.f || v_exit < 0.f) ? (v_exit - v_entry) : max(v_exit, -v_entry) )
          : // v_exit <= v_entry                coasting             axis reversal
            ( (v_entry < 0.f || v_exit > 0.f) ? (v_entry - v_exit) : max(-v_exit, v_entry) );

      if (jerk > max_jerk[axis]) {
        v_factor *= max_jerk[axis] / jerk;
        ++limited;
      }
    }
    if (limited) vmax_junction *= v_factor;
    // Now the transition velocity is known, which maximizes the shared exit / entry velocity while
    // respecting the jerk factors, it may be possible, that applying separate safe exit / entry velocities will achieve faster prints.
    const float vmax_junction_threshold = vmax_junction * 0.99f;
    if (previous_safe_speed > vmax_junction_threshold && safe_speed > vmax_junction_threshold) {
      // Not coasting. The machine will stop and start the movements anyway,
      // better to start the segment from start.
      SBI(block->flag, BLOCK_BIT_START_FROM_FULL_HALT);
      vmax_junction = safe_speed;
    }
  }
  else {
    SBI(block->flag, BLOCK_BIT_START_FROM_FULL_HALT);
    vmax_junction = safe_speed;
  }

  // Max entry speed of this block equals the max exit speed of the previous block.
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  const float v_allowable = max_allowable_speed(-block->acceleration, MINIMUM_PLANNER_SPEED, block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  block->flag |= BLOCK_FLAG_RECALCULATE | (block->nominal_speed <= v_allowable ? BLOCK_FLAG_NOMINAL_LENGTH : 0);

  // Update previous path unit_vector and nominal speed
  COPY(previous_speed, current_speed);
  previous_nominal_speed = block->nominal_speed;
  previous_safe_speed = safe_speed;

  calculate_trapezoid_for_block(block, block->entry_speed / block->nominal_speed, safe_speed / block->nominal_speed);

  // Move buffer head
  block_buffer_head = next_buffer_head;

  // Update the position (only when a move was queued)
  COPY(position, target);

  recalculate();

  stepper.wake_up();

} // buffer_line()

/**
 * Directly set the planner XYZ position (and stepper positions)
 * converting mm (or angles for SCARA) into steps.
 *
 * On CORE machines stepper ABC will be translated from the given XYZ.
 */

void Planner::_set_position_mm(const float &a, const float &b, const float &c, const float &e) {
  #define _EINDEX E_AXIS
  long na = position[X_AXIS] = LROUND(a * axis_steps_per_mm[X_AXIS]),
       nb = position[Y_AXIS] = LROUND(b * axis_steps_per_mm[Y_AXIS]),
       nc = position[Z_AXIS] = LROUND(c * axis_steps_per_mm[Z_AXIS]),
       ne = position[E_AXIS] = LROUND(e * axis_steps_per_mm[_EINDEX]);
  stepper.set_position(na, nb, nc, ne);
  previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
  ZERO(previous_speed);
}

// Recalculate the steps/s^2 acceleration rates, based on the mm/s^2
void Planner::reset_acceleration_rates() {
  uint32_t highest_rate = 1;
  LOOP_XYZE_N(i) {
    max_acceleration_steps_per_s2[i] = max_acceleration_mm_per_s2[i] * axis_steps_per_mm[i];
    NOLESS(highest_rate, max_acceleration_steps_per_s2[i]);
  }
  cutoff_long = 4294967295UL / highest_rate;
}

// Recalculate position, steps_to_mm if axis_steps_per_mm changes!
void Planner::refresh_positioning() {
  LOOP_XYZE_N(i) steps_to_mm[i] = 1.0 / axis_steps_per_mm[i];
  set_position_mm_kinematic(current_position);
  reset_acceleration_rates();
}

void Planner::set_position_mm_kinematic(const float position[NUM_AXIS]) {
  const float * const lpos = position;
  _set_position_mm(lpos[X_AXIS], lpos[Y_AXIS], lpos[Z_AXIS], position[E_AXIS]);
}

