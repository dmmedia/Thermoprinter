#pragma once

/*
 * CommandProcessor.h
 *
 *  Created on: 24. dets 2017
 *      Author: Den
 */

namespace CommandProcessor {
	extern uint8_t commands_in_queue; // Count of commands in the queue
	extern uint8_t cmd_queue_index_r; // Ring buffer read position
	extern uint8_t cmd_queue_index_w; // Ring buffer write position

	extern char command_queue[BUFSIZE][MAX_CMD_SIZE];

	// Number of characters read in the current line of serial input
	extern int32_t serial_count;

	extern bool send_ok[BUFSIZE];

    extern uint32_t current_position;
	extern uint32_t destination;

	extern uint32_t gcode_LastN;
	extern uint32_t gcode_N;

	void gcode_get_destination();
	void sync_plan_position();
	void prepare_move_to_destination();

	void do_blocking_move_to(const float &lm, const float &fr_mm_s/*=0.0*/);

	void process_next_command();

	void FlushSerialRequestResend();
	void gcode_line_error(bool doFlush = true);

	void get_available_commands();

	void ok_to_send();

	void init(void);

	void process(void);

	extern millis_t previous_cmd_ms;
	inline void refresh_cmd_timeout() { previous_cmd_ms = GetTick(); }

} // namespace CommandProcessor
