/*
 * CommandProcessor.cpp
 *
 *  Created on: 24. dets 2017
 *      Author: Den
 */

#include <stm32l0xx.h>
#include "typedefs.h"
#include "macros.h"
#include "gpio.h"
#include "Configuration.h"
#include <stdint.h>
#include "main.h"
#include "rcc.h"
#include <CommandProcessor.h>
#include <stddef.h>
#include <stdlib.h>
#include "CommandParser.h"
#include "PLanner.h"
#include "Stepper.h"
#include "serial.h"
#include "string.h"
#include "Settings.h"
#include "Thermoprinter.h"

namespace CommandProcessor {
	//
	// Private definitions
	//

	#define MMS_SCALED(MM_S) (((MM_S) * RuntimeSettings::feedrate_percentage) * 0.01)

	//
	// Private variables
	//

	static uint8_t commands_in_queue = 0U; // Count of commands in the queue
	static uint8_t cmd_queue_index_r = 0U; // Ring buffer read position
	static uint8_t cmd_queue_index_w = 0U; // Ring buffer write position

	static char command_queue[BUFSIZE][MAX_CMD_SIZE];

	// Number of characters read in the current line of serial input
	static uint32_t serial_count = 0U;

	static bool send_ok[BUFSIZE];

	//
	// Cartesian Destination
	//   A temporary position, usually applied to 'current_position'.
	//   Set with 'gcode_get_destination' or 'set_destination_to_current'.
	//   'line_to_destination' sets 'current_position' to 'destination'.
	//
	static uint32_t destination = 0U;

	static uint32_t gcode_LastN = 0U;
	static uint32_t gcode_N = 0U;

	//
	// Private prototypes
	//

	static void gcode_get_destination();

	static void sync_plan_position();

	static void prepare_move_to_destination();

	static void process_next_command();

	static void flushSerialRequestResend();

	static void gcode_line_error(bool doFlush = true);

	static void get_available_commands();

	static void ok_to_send();

	static void line_to_destination(const float32_t fr_mm_s);
	static void line_to_destination();

	static inline bool prepare_move_to_destination_cartesian();

	static inline void set_current_to_destination();

	static inline void gcode_M0();

	static inline void gcode_G92();

	static inline void gcode_P0();

	static inline void commitCommand(bool say_ok);

	static inline bool enqueueCommand(const char* cmd, bool say_ok = false);

	static inline void get_serial_commands();

	//
	// Public variable initialization
	//

	//
	// Cartesian Current Position
	//   Used to track the logical position as moves are queued.
	//   Used by 'line_to_current_position' to do a move after changing it.
	//   Used by 'SYNC_PLAN_POSITION_KINEMATIC' to update 'planner.position'.
	//
	uint32_t current_position = 0U;

	//
	// Namespace body
	//

	//
	// Set destination and feedrate from the current GCode command
	//
	//  - Set destination from command line
	//  - Set to current for non-movement commands
	//
	static void gcode_get_destination() {
		if (CommandParser::command_letter == 'M') {
		    destination = CommandParser::value_axis_units() + current_position;
		} else if (CommandParser::command_letter == 'P') {
			destination = 2U + current_position;
		} else {
			destination = current_position;
		}
	}

	//
	// Move the planner to the position stored in the destination, which is
	// used by M0 function to set a destination.
	//
	static void line_to_destination(const float32_t fr_mm_s) {
	    Planner::buffer_line(destination, fr_mm_s);
	}

	static void line_to_destination() {
		line_to_destination(RuntimeSettings::feedrate_mm_s);
	}

	//
	// sync_plan_position
	//
	// Set the planner/stepper positions directly from current_position with
	// no kinematic translation. Used for syncing.
	//
	static void sync_plan_position() {
		Planner::set_position_mm(current_position);
	}

	//
	// Prepare a linear move in a Cartesian setup.
	// If Mesh Bed Leveling is enabled, perform a mesh move.
	//
	// Returns true if the caller didn't update current_position.
	//
	static inline bool prepare_move_to_destination_cartesian() {
	    // Do not use feedrate_percentage for E or Z only moves
	    if (current_position == destination)
	        line_to_destination();
	    else {
	        const float fr_scaled = MMS_SCALED(RuntimeSettings::feedrate_mm_s);
	            line_to_destination(fr_scaled);
	      }
	    return false;
	}

	static inline void set_current_to_destination() {
		current_position = destination;
	}

	//
	// Prepare a single move and get ready for the next one
	//
	// This may result in several calls to planner.buffer_line to
	// do smaller moves for DELTA, SCARA, mesh moves, etc.
	//
	static void prepare_move_to_destination() {
		if (!prepare_move_to_destination_cartesian()) {
			set_current_to_destination();
		}
	}

	//
	// M0: Stepped movement of motor
	//
	static inline void gcode_M0() {
		if (Thermoprinter::IsRunning()) {
			gcode_get_destination();

			prepare_move_to_destination();
		}
	}

	//
	// G92: Set current position to 0
	//
	static inline void gcode_G92() {
		Stepper::synchronize();

		current_position = 0;

		sync_plan_position();
	}

	static inline void gcode_P0() {
		// TODO: add parser.byte_arg to buffer
	}

	//
	// Process a single command and dispatch it to its handler
	// This is called from the main loop()
	//
	static void process_next_command() {
		char* const current_command = command_queue[cmd_queue_index_r];

		Serial::SERIAL_ECHO_START();
		SERIAL_ECHOLN(current_command);

		// Parse the next command in the queue
		CommandParser::parse(current_command);

		// Handle a known M, or P
		switch (CommandParser::command_letter) {
	    	case 'M':
	    		switch (CommandParser::codenum) {
	    			case 0:
	    				gcode_M0();
	    				break;

	    			case 90: // G90
	    				RuntimeSettings::relative_mode = false;
	    				break;
	    			case 91: // G91
	    				RuntimeSettings::relative_mode = true;
	    				break;

	    			case 92: // G92
	    				gcode_G92();
	    				break;
	    		}
	    		break;

	    	case 'P':
	    		switch (CommandParser::codenum) {
	    			case 0: // P0
	    				gcode_P0();
	    				break;
	    		}
	    		break;

    		default:
    			CommandParser::unknown_command_error();
		}

		ok_to_send();
	}

	//
	// Send a "Resend: nnn" message to the host to
	// indicate that a command needs to be re-sent.
	//
	static void flushSerialRequestResend() {
		MarlinSerial::flush();
		SERIAL_PROTOCOLPGM(MSG_RESEND);
		SERIAL_PROTOCOLLN(gcode_LastN + 1);
		ok_to_send();
	}

	static void gcode_line_error(bool doFlush) {
		if (doFlush) {
			flushSerialRequestResend();
		}
		serial_count = 0U;
	}

	//
	// Once a new command is in the ring buffer, call this to commit it
	//
	static inline void commitCommand(bool say_ok) {
		send_ok[cmd_queue_index_w] = say_ok;
		if (++cmd_queue_index_w >= BUFSIZE) {
			cmd_queue_index_w = 0U;
		}
		commands_in_queue++;
	}

	//
	// Copy a command from RAM into the main command buffer.
	// Return true if the command was successfully added.
	// Return false for a full buffer, or if the 'command' is a comment.
	//
	static inline bool enqueueCommand(const char* cmd, bool say_ok) {
		bool res = false;
		if ((*cmd != ';') && (commands_in_queue < BUFSIZE)) {
			strcpy(command_queue[cmd_queue_index_w], cmd);
			commitCommand(say_ok);
			res = true;
		}
		return res;
	}

	//
	// Get all commands waiting on the serial port and queue them.
	// Exit when the buffer is full or when no more characters are
	// left on the serial port.
	//
	static inline void get_serial_commands() {
		static char serial_line_buffer[MAX_CMD_SIZE] { };

		//
		// Loop while serial characters are incoming and the queue is not full
		//
		while (commands_in_queue < BUFSIZE && MarlinSerial::available() > 0) {

			char serial_char = MarlinSerial::read();

			//
			// If the character ends the line
			//
			if ((serial_char == '\n') || (serial_char == '\r')) {

				if (!serial_count) {
					continue; // skip empty lines
				}

				serial_line_buffer[serial_count] = '\0'; // terminate string
				serial_count = 0U; //reset buffer

				char* command = serial_line_buffer;

				while (*command == ' ') {
					command++; // skip any leading spaces
				}

				char *npos = (*command == 'N') ? command : nullptr;

				if (npos) {
					gcode_N = strtol(npos + 1U, nullptr, 10);

					if (gcode_N != gcode_LastN + 1U) {
						gcode_line_error(MSG_ERR_LINE_NO);
						return;
					}

					gcode_LastN = gcode_N;
				}
				// Add the command to the queue
				enqueueCommand(serial_line_buffer, true);
			}
			else if (serial_count >= MAX_CMD_SIZE - 1) {
				// Keep fetching, but ignore normal characters beyond the max length
				// The command will be injected when EOL is reached
			}
			else if (serial_char == '\\') {  // Handle escapes
				if (MarlinSerial::available() > 0) {
					// if we have one more character, copy it over
					serial_char = MarlinSerial::read();
					serial_line_buffer[serial_count++] = serial_char;
				}
				// otherwise do nothing
			}
			else { // it's not a newline, carriage return or escape char
				serial_line_buffer[serial_count++] = serial_char;
			}

		} // queue has space, serial has data
	}

	//
	// Add to the circular command queue the next command from:
	//  - The command-injection queue (injected_commands_P)
	//  - The active serial input (usually USB)
	//
	static void get_available_commands() {
		get_serial_commands();
	}

	//
	// Send an "ok" message to the host, indicating
	// that a command was successfully processed.
	//
	// If ADVANCED_OK is enabled also include:
	//   N<int>  Line number of the command, if any
	//   P<int>  Planner space remaining
	//   B<int>  Block queue space remaining
	//
	static void ok_to_send() {
		if (send_ok[cmd_queue_index_r]) {
			SERIAL_PROTOCOLPGM(MSG_OK);
		  	#ifdef ADVANCED_OK
				char* p = command_queue[cmd_queue_index_r];
				if (*p == 'N') {
					SERIAL_PROTOCOL(' ');
					SERIAL_ECHO(*p++);
					while (NUMERIC_SIGNED(*p)) {
						SERIAL_ECHO(*p++);
					}
				}
				SERIAL_PROTOCOLPGM(" P"); SERIAL_PROTOCOL(int(BLOCK_BUFFER_SIZE - planner.movesplanned() - 1));
				SERIAL_PROTOCOLPGM(" B"); SERIAL_PROTOCOL(BUFSIZE - commands_in_queue);
		  	#endif
			SERIAL_EOL();
		}
	}

	void init(void) {
		// Send "ok" after commands by default
		for (uint8_t i = 0U; i < BUFSIZE; i++) {
			send_ok[i] = true;
		}

		// Vital to init stepper/planner equivalent for current_position
		sync_plan_position();
	}

	void process(void) {
		if (commands_in_queue < BUFSIZE) {
			get_available_commands();
		}

		if (commands_in_queue) {
			process_next_command();

			// The queue may be reset by a command handler or by code invoked by idle() within a handler
			if (commands_in_queue) {
			    --commands_in_queue;
			    if ((++cmd_queue_index_r) >= BUFSIZE) {
			    	cmd_queue_index_r = 0U;
			    }
			}
		}
	}

	// End

} // namespace CommandProcessor
