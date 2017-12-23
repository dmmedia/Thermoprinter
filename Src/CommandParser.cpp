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
#include "serial.h"
#include <cstring>

namespace CommandParser {
	char *value_ptr;           // Set by seen, used to fetch the value

	#if ENABLED(FASTER_COMMAND_PARSER)
		uint8_t codebits[4];        // Parameters pre-scanned
		uint8_t param[26];       // For A-Z, offsets into command args
	#else
		char *command_args;      // Args start here, for slow scan
	#endif

	// Command line state
	char *command_ptr;               // The command, so it can be echoed
	char *string_arg;                // string of command line
	char *int_arg;                   // signed integer of comman line

	//	std::string stringArg;

	char command_letter;             // M, or P
	uint8_t codenum;                     // 123

	//
	// Clear all code-seen (and value pointers)
	//
	// Since each param is set/cleared on seen codes,
	// this may be optimized by commenting out ZERO(param)
	//
	void reset() {
	  string_arg = NULL;                    // No whole line argument
	//  stringArg = "";
	  command_letter = '?';                 // No command letter
	  codenum = 0U;                         // No command code
	  #if ENABLED(FASTER_COMMAND_PARSER)
		ZERO(codebits);                     // No codes yet
	  #endif
	}

	void parse(char *p) {
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

    // Code seen bit was set. If not found, value_ptr is unchanged.
    // This allows "if (seen('A')||seen('B'))" to use the last-found value.
    bool seen(const char c) {
        const uint8_t ind = LETTER_OFF(c);
        bool b = false;
        if (ind < COUNT(param)) // Only A-Z
        {
			b = TEST(codebits[PARAM_IND(ind)], PARAM_BIT(ind));
			if (b) {
				value_ptr = (param[ind] != 0U) ? (&command_ptr[param[ind]]) : nullptr;
			}
        }

        return b;
    }

    // The code value pointer was set
    FORCE_INLINE bool has_value() { return value_ptr != nullptr; }

    bool seen_any() {
    	return (codebits[3U] != 0U) ||
    			(codebits[2U] != 0U) ||
				(codebits[1U] != 0U) ||
				(codebits[0U] != 0U);
    }

    // Seen a parameter with a value
    inline bool seenval(const char c) { return seen(c) && has_value(); }

}
