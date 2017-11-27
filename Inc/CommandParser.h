/*
 * CommandParser.h
 *
 *  Created on: 14. okt 2017
 *      Author: Den
 */

#ifndef COMMANDPARSER_H_
#define COMMANDPARSER_H_

#include <stdint.h>
#include "macros.h"
#include "Configuration.h"
#include <stdlib.h>

class CommandParser {
	static char *value_ptr;           // Set by seen, used to fetch the value

#if ENABLED(FASTER_COMMAND_PARSER)
    static uint8_t codebits[4];        // Parameters pre-scanned
    static uint8_t param[26];       // For A-Z, offsets into command args
#else
    static char *command_args;      // Args start here, for slow scan
#endif

public:
	// Reset is done before parsing
	static void reset();

	static void parse(char * p);

	// Command line state
	static char *command_ptr,               // The command, so it can be echoed
	            *string_arg;                // string of command line

    static char command_letter;             // M, or P
    static int codenum;                     // 123

	#define PARAM_IND(N)  ((N) >> 3)
	#define PARAM_BIT(N)  ((N) & 0x7)
	#define LETTER_OFF(N) ((N) - 'A')

#if ENABLED(FASTER_COMMAND_PARSER)

  // Set the flag and pointer for a parameter
  static void set(const char c, char * const ptr) {
    const uint8_t ind = LETTER_OFF(c);
    if (ind >= COUNT(param)) return;           // Only A-Z
    SBI(codebits[PARAM_IND(ind)], PARAM_BIT(ind));        // parameter exists
    param[ind] = ptr ? ptr - command_ptr : 0;  // parameter offset or 0
  }

  // Code seen bit was set. If not found, value_ptr is unchanged.
  // This allows "if (seen('A')||seen('B'))" to use the last-found value.
  static bool seen(const char c) {
    const uint8_t ind = LETTER_OFF(c);
    if (ind >= COUNT(param)) return false; // Only A-Z
    const bool b = TEST(codebits[PARAM_IND(ind)], PARAM_BIT(ind));
    if (b) value_ptr = param[ind] ? command_ptr + param[ind] : (char*)NULL;
    return b;
  }

  static bool seen_any() { return codebits[3] || codebits[2] || codebits[1] || codebits[0]; }

  #define SEEN_TEST(L) TEST(codebits[LETTER_IND(L)], LETTER_BIT(L))

#else // !FASTER_GCODE_PARSER

  // Code is found in the string. If not found, value_ptr is unchanged.
  // This allows "if (seen('A')||seen('B'))" to use the last-found value.
  static bool seen(const char c) {
    const char *p = strchr(command_args, c);
    const bool b = !!p;
    if (b) value_ptr = DECIMAL_SIGNED(p[1]) ? &p[1] : (char*)NULL;
    return b;
  }

  static bool seen_any() { return *command_args == '\0'; }

  #define SEEN_TEST(L) !!strchr(command_args, L)

#endif // !FASTER_GCODE_PARSER

  // Float removes 'E' to prevent scientific notation interpretation
  inline static float value_float() {
    if (value_ptr) {
      char *e = value_ptr;
      for (;;) {
        const char c = *e;
        if (c == '\0' || c == ' ') break;
        if (c == 'E' || c == 'e') {
          *e = '\0';
          const float ret = strtod(value_ptr, NULL);
          *e = c;
          return ret;
        }
        ++e;
      }
      return strtod(value_ptr, NULL);
    }
    return 0.0;
  }

  FORCE_INLINE static float value_linear_units()                  {            return value_float(); }
  FORCE_INLINE static float value_axis_units()    { return value_float(); }

  FORCE_INLINE static float    linearval(const char c, const float dval=0.0)  { return seenval(c) ? value_linear_units() : dval; }

  FORCE_INLINE static float value_feedrate() { return value_linear_units(); }

  // Seen a parameter with a value
  inline static bool seenval(const char c) { return seen(c) && has_value(); }

  // The code value pointer was set
  FORCE_INLINE static bool has_value() { return value_ptr != NULL; }

};

extern CommandParser parser;

#endif /* COMMANDPARSER_H_ */
