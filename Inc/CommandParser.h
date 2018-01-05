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
	//
	// Public definitions and constants
	//

	//
	// Public variables
	//

	extern char command_letter;             // M, or P

    extern uint8_t codenum;                     // 123

	//
	// Public functions
	//

	void parse(char* p);

    int32_t value_axis_units();

    void unknown_command_error();

    // End

}
