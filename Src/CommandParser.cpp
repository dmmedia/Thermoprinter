/*
 * CommandParser.cpp
 *
 *  Created on: 14. okt 2017
 *      Author: Den
 */

#include "CommandParser.h"
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
     *CommandParser::int_arg,
     *CommandParser::value_ptr;
char CommandParser::command_letter;
uint8_t CommandParser::codenum;

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

  // *p now points to the current command, which should be M, or P
  command_ptr = p;

  // Get the command letter, which must be M, or P
  const char letter = *p++;

  // Bail if the letter is not M, or P
  switch (letter) { case 'M': case 'P': break; default: return; }

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
  switch (letter) {
    // Only use string_arg for these P codes
    case 'P':
  	  switch (codenum) {
  	  	case 0:
  	  	  int_arg = nullptr;
          string_arg = p;
  	  	  // sanity check
  	  	  if (strlen(string_arg) != 96) {
  	  		// skip invalid argument
  	  	  }
  	  	  return;
  	  	default:
  	  	  break;
  	  }
  	  break;
  	case 'M':
  	  switch (codenum) {
  	    case 0: {
  	      string_arg = nullptr;
  	      const bool has_num = DECIMAL_SIGNED(*p);  // The parameter has a number [-+0-9.]
  	      if (has_num) {
            int_arg = p;
  	      }
  	      return;
  	    }
  	    default:
  	      break;
  	  }
  	  break;
    default:
      break;
  }
}
