/*
 * CommandParser.cpp
 *
 *  Created on: 14. okt 2017
 *      Author: Den
 */

#include <CommandParser.h>
#include "main.h"

#if ENABLED(FASTER_COMMAND_PARSER)
  // Optimized Parameters
  uint8_t CommandParser::codebits[4];   // found bits
  uint8_t CommandParser::param[26];  // parameter offsets from command_ptr
#else
  char *CommandParser::command_args; // start of parameters
#endif

// Create a global instance of the GCode parser singleton
CommandParser parser;

char *CommandParser::command_ptr,
     *CommandParser::string_arg,
     *CommandParser::value_ptr;
char CommandParser::command_letter;
int CommandParser::codenum;

/**
 * Clear all code-seen (and value pointers)
 *
 * Since each param is set/cleared on seen codes,
 * this may be optimized by commenting out ZERO(param)
 */
void CommandParser::reset() {
  string_arg = NULL;                    // No whole line argument
  command_letter = '?';                 // No command letter
  codenum = 0;                          // No command code
  #if ENABLED(FASTER_COMMAND_PARSER)
    ZERO(codebits);                     // No codes yet
  #endif
}

void CommandParser::parse(char *p) {

  reset(); // No codes to report

  // Skip spaces
  while (*p == ' ') ++p;

  // Skip N[-0-9] if included in the command line
  if (*p == 'N' && NUMERIC_SIGNED(p[1])) {
    p += 2;                  // skip N[-0-9]
    while (NUMERIC(*p)) ++p; // skip [0-9]*
    while (*p == ' ')   ++p; // skip [ ]*
  }

  // *p now points to the current command, which should be M, or P
  command_ptr = p;

  // Get the command letter, which must be M, or P
  const char letter = *p++;

  // Nullify asterisk and trailing whitespace
  char *starpos = strchr(p, '*');
  if (starpos) {
    --starpos;                          // *
    while (*starpos == ' ') --starpos;  // spaces...
    starpos[1] = '\0';
  }

  // Bail if the letter is not M, or P
  switch (letter) { case 'M': case 'P': break; default: return; }

  // Skip spaces to get the numeric part
  while (*p == ' ') p++;

  // Bail if there's no command code number
  if (!NUMERIC(*p)) return;

  // Save the command letter at this point
  // A '?' signifies an unknown command
  command_letter = letter;

  // Get the code number - integer digits only
  codenum = 0;
  do {
    codenum *= 10, codenum += *p++ - '0';
  } while (NUMERIC(*p));

  // Skip all spaces to get to the first argument, or nul
  while (*p == ' ') p++;

  // The command parameters (if any) start here, for sure!

  #if DISABLED(FASTER_COMMAND_PARSER)
    command_args = p; // Scan for parameters in seen()
  #endif

  // Only use string_arg for these M codes
  if (letter == 'M') switch (codenum) { case 0: string_arg = p; return; default: break; }

  /**
   * Find all parameters, set flags and pointers for fast parsing
   *
   * Most codes ignore 'string_arg', but those that want a string will get the right pointer.
   * The following loop assigns the first "parameter" having no numeric value to 'string_arg'.
   */
  string_arg = NULL;
  while (char code = *p++) {                    // Get the next parameter. A NUL ends the loop

    // Arguments MUST be uppercase for fast GCode parsing
    #if ENABLED(FASTER_COMMAND_PARSER)
      #define PARAM_TEST WITHIN(code, 'A', 'Z')
    #else
      #define PARAM_TEST true
    #endif

    if (PARAM_TEST) {

      while (*p == ' ') p++;                    // Skip spaces between parameters & values
      const bool has_num = DECIMAL_SIGNED(*p);  // The parameter has a number [-+0-9.]

      if (!has_num && !string_arg) {            // No value? First time, keep as string_arg
        string_arg = p - 1;
      }

      #if ENABLED(FASTER_COMMAND_PARSER)
        set(code, has_num ? p : NULL);          // Set parameter exists and pointer (NULL for no number)
      #endif
    }
    else if (!string_arg) {                     // Not A-Z? First time, keep as the string_arg
      string_arg = p - 1;
    }

    if (!WITHIN(*p, 'A', 'Z')) {
      while (*p && NUMERIC(*p)) p++;            // Skip over the value section of a parameter
      while (*p == ' ') p++;                    // Skip over all spaces
    }
  }
}
