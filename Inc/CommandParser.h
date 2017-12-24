/*
 * CommandParser.h
 *
 *  Created on: 14. okt 2017
 *      Author: Den
 */

#pragma once

#include "macros.h"
#include "Configuration.h"

namespace CommandParser {
	extern char *value_ptr;           // Set by seen, used to fetch the value

#if ENABLED(FASTER_COMMAND_PARSER)
    extern uint8_t codebits[4];        // Parameters pre-scanned
    extern uint8_t param[26];       // For A-Z, offsets into command args
#else
    extern char *command_args;      // Args start here, for slow scan
#endif

	// Command line state
    extern char *command_ptr;               // The command, so it can be echoed
	extern char *string_arg;                // string of command line
	extern char *int_arg;                   // signed integer of comman line

//	extern std::string stringArg;

    extern char command_letter;             // M, or P
    extern uint8_t codenum;                     // 123

	// Reset is done before parsing
	void reset();

	void parse(char * p);

	#define LETTER_OFF(N) (static_cast<uint8_t>(N) - static_cast<uint8_t>('A'))

    // Code seen bit was set. If not found, value_ptr is unchanged.
    // This allows "if (seen('A')||seen('B'))" to use the last-found value.
    bool seen(const char c);

    bool seen_any();

    #define SEEN_TEST(L) TEST(codebits[LETTER_IND(L)], LETTER_BIT(L))

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

    FORCE_INLINE int32_t value_linear_units() { return value_int(); }
    FORCE_INLINE int32_t value_axis_units() { return value_int(); }

    // The code value pointer was set
    FORCE_INLINE bool has_value();

    // Seen a parameter with a value
    inline bool seenval(const char c);

    FORCE_INLINE float32_t linearval(const char c, const float32_t dval = 0.0)  { return seenval(c) ? value_linear_units() : dval; }

    FORCE_INLINE float32_t value_feedrate() { return value_linear_units(); }

    void unknown_command_error();
}
