#pragma once

/*
 * CommandProcessor.h
 *
 *  Created on: 24. dets 2017
 *      Author: Den
 */

namespace CommandProcessor {
	//
	// Public definitions and constants
	//

	//
	// Public variables
	//

	extern uint32_t current_position;

	//
	// Public functions
	//

	void init(void);

	void process(void);

	// End

} // namespace CommandProcessor
