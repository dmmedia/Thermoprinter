#pragma once

/*
 * Temperature.h
 *
 *  Created on: 9. nov 2017
 *      Author: Den
 */

namespace AdcManager {
	//
	// Public constants and definitions
	//

	//
	// Public variables
	//

	extern Timers::TIM_HandleTypeDef htim2;

	extern volatile bool in_temp_isr;

	//
	// Public functions
	//

	void init();

	//
	// Switch off all heaters, set all target temperatures to 0
	///
	void disable_all_heaters();

	// End

}
