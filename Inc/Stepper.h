/*
 * Stepper.h
 *
 *  Created on: 7. nov 2017
 *      Author: Den
 */

#ifndef STEPPER_H_
#define STEPPER_H_

//#include "macros.h"
//#include "Conditionals.h"
//#include "Planner.h"

const uint16_t speed_lookuptable_fast[256][2] = {
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

const uint16_t speed_lookuptable_slow[256][2] = {
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

#define MOTOR_DIR_INIT SET_OUTPUT(MOTOR_DIR)
#define MOTOR_DIR_WRITE(STATE) WRITE(MOTOR_DIR,STATE)
#define MOTOR_DIR_READ READ(MOTOR_DIR)

#define MOTOR_STEP_INIT SET_OUTPUT(MOTOR_STEP)
#define MOTOR_STEP_WRITE(STATE) WRITE(MOTOR_STEP,STATE)
#define MOTOR_STEP_READ READ(MOTOR_STEP)

static unsigned short MultiU16X8toH16(unsigned char charIn1, unsigned int intIn2) {
	return (unsigned short)((intIn2 * charIn1) >> 16);
}

class Stepper;
extern Stepper stepper;

TIM_HandleTypeDef htim6;

class Stepper {
public:
    static block_t* current_block;  // A pointer to the block currently being traced

    Stepper() {};

    //
    // Interrupt Service Routines
    //
    static void isr();

    //
    // The stepper subsystem goes to sleep when it runs out of things to execute. Call this
    // to notify the subsystem that it is time to go to work.
    //
    static void wake_up();

    //
    // Block until all buffered steps are executed
    //
    static void synchronize();

    //
    // Set the current position in steps
    //
    static void set_position(const long &a);

    //
    // The direction of a single motor
    //
    static FORCE_INLINE bool motor_direction() { return TEST(last_direction_bits, 0); }

    //
    // Set direction bits for all steppers
    //
    static void set_directions();

    //
    // Initialize stepper hardware
    //
    static void init();

    //
    // Handle a triggered endstop
    //
    static void endstop_triggered();

    static inline void kill_current_block() {
      step_events_completed = current_block->step_event_count;
    }

private:

    // Counter variables for the Bresenham line tracer
    static long counter_MOTOR;
    static volatile uint32_t step_events_completed; // The number of step events executed in the current block

    static FORCE_INLINE unsigned short calc_timer(unsigned short step_rate) {
      unsigned short timer;

      NOMORE(step_rate, MAX_STEP_FREQUENCY);

      if (step_rate > 20000) { // If steprate > 20kHz >> step 4 times
        step_rate >>= 2;
        step_loops = 4;
      }
      else if (step_rate > 10000) { // If steprate > 10kHz >> step 2 times
        step_rate >>= 1;
        step_loops = 2;
      }
      else {
        step_loops = 1;
      }

      NOLESS(step_rate, SystemCoreClock / 500000);
      step_rate -= SystemCoreClock / 500000; // Correct for minimal speed
      if (step_rate >= (8 * 256)) { // higher step rate
        uint16_t *table_address = (uint16_t *)&speed_lookuptable_fast[(uint8_t) (step_rate >> 8)][0];
        unsigned char tmp_step_rate = (step_rate & 0x00FF);
        uint16_t gain = *(table_address + 1);
        timer = MultiU16X8toH16(tmp_step_rate, gain);
        timer = (*table_address) - timer;
      }
      else { // lower step rates
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

    //
    // Positions of stepper motors, in step units
    //
    static volatile long count_position;

    static uint8_t last_direction_bits;        // The next stepping-bits to be output
    static uint16_t cleaning_buffer_counter;

    static long acceleration_time, deceleration_time;
    static unsigned short acc_step_rate; // needed for deceleration start point
    static uint8_t step_loops, step_loops_nominal;
    static unsigned short TIM6_ARR_nominal;

    //
    // Current direction of stepper motors (+1 or -1)
    //
    static volatile signed char count_direction;

    // Initialize the trapezoid generator from the current block.
    // Called whenever a new block begins.
    static FORCE_INLINE void trapezoid_generator_reset() {
      static int8_t last_extruder = -1;

      if (current_block->direction_bits != last_direction_bits || current_block->active_extruder != last_extruder) {
        last_direction_bits = current_block->direction_bits;
        last_extruder = current_block->active_extruder;
        set_directions();
      }

      deceleration_time = 0;
      // step_rate to timer interval
      TIM6_ARR_nominal = calc_timer(current_block->nominal_rate);
      // make a note of the number of step loops required at nominal speed
      step_loops_nominal = step_loops;
      acc_step_rate = current_block->initial_rate;
      acceleration_time = calc_timer(acc_step_rate);
      __HAL_TIM_SET_AUTORELOAD(&htim6, acceleration_time);
    }

    static volatile long endstops_trigsteps;
};

#endif /* STEPPER_H_ */
