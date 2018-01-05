//
// CommandParser.cpp
//
//  Created on: 14. okt 2017
//      Author: Den
//

#include <stdint.h>
#include <cstdlib>
#include <stm32l0xx.h>
#include "typedefs.h"
#include "CommandParser.h"
#include "main.h"
#include "rcc.h"
#include "SREGEmulation.h"
#include "serial.h"
#include <cstring>

namespace CommandParser {
	//
	// Private variables
	//

    // Command line state
    static char *command_ptr;               // The command, so it can be echoed
	static char *string_arg;                // string of command line
	static char *int_arg;                   // signed integer of comman line

	// static std::string stringArg;

	//
	// Private function declarations
	//

	// Reset is done before parsing
	static void reset();

    inline int value_int();

    //
    // Public variable initialization
    //

	char command_letter = '\0';             // M, or P
	uint8_t codenum = 0U;                     // 123

    //
	// Namespace body
	//

	//
	// Clear all code-seen (and value pointers)
	//
	// Since each param is set/cleared on seen codes
	//
	static void reset() {
        string_arg = NULL;                    // No whole line argument
        // stringArg = "";
	    command_letter = '?';                 // No command letter
	    codenum = 0U;                         // No command code
	}

	void parse(char* p) {
		reset(); // No codes to report

		// Skip spaces
		while (*p == ' ') {++p;}

		// *p now points to the current command, which should be M, or P
		command_ptr = p;

		// Get the command letter, which must be M, or P
		const char letter = *p;
		p++;

		// Bail if the letter is not M, or P or if there's no command code number
		if (((letter == 'M') || (letter == 'P')) && NUMERIC(*p)) {
			// Save the command letter at this point
			// A '?' signifies an unknown command
			command_letter = letter;

			// Get the code number - integer digits only
			codenum = 0U;
			do {
				codenum *= 10U;
				codenum += static_cast<uint8_t>(*p) - static_cast<uint8_t>('0');
				p++;
			} while (NUMERIC(*p));

			// Skip all spaces to get to the first argument, or nul
			while (*p == ' ') {p++;}

			// The command parameters (if any) start here, for sure!
			switch (letter) {
				// Only use string_arg for these P codes
				case 'P':
					if (codenum == 0U) {
						int_arg = nullptr;
						string_arg = p;
	//    				stringArg = p;
						// sanity check
						//lint -esym(586, strlen)
						if (strlen(string_arg) != 96U) {
	//    				if (stringArg.length() != 96U) {
							// skip invalid argument
						}
					}
					break;
				case 'M':
					if (codenum == 0U) {
						string_arg = nullptr;
	//	    			stringArg = "";
						const bool has_num = DECIMAL_SIGNED(*p);  // The parameter has a number [-+0-9.]
						if (has_num) {
							int_arg = p;
						}
					}
					break;
				default:
					// Skip any other commands
					break;
			}
		}
	}

	void unknown_command_error() {
	  SERIAL_ECHO_START();
	  SERIAL_ECHOPAIR("Unknown command: \"", command_ptr);
	  SERIAL_CHAR(static_cast<uint8_t>('"'));
	  SERIAL_EOL();
	}

    int32_t value_axis_units() {
    	return value_int();
    }

    // Float removes 'E' to prevent scientific notation interpretation
    inline int value_int() {
    	if (int_arg) {
    		char *e = int_arg;
    		for (;;) {
    			const char c { *e };
    			if (c == '\0' || c == ' ') break;
    			if (c == 'E' || c == 'e') {
    				*e = '\0';
    				const int ret = strtol(int_arg, NULL, 10);
    				*e = c;
    				return ret;
    			}
    			++e;
    		}
    		return strtol(int_arg, NULL, 10);
    	}
    	return 0;
    }

    // End

}
