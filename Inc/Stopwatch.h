/*
 * Stopwatch.h
 *
 *  Created on: 14. nov 2017
 *      Author: Den
 */

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

class Stopwatch {
public:
	Stopwatch();

    /**
     * @brief Stops the stopwatch
     * @details Stops the running timer, it will silently ignore the request if
     * no timer is currently running.
     * @return true is method was successful
     */
    bool stop();

};

#endif /* STOPWATCH_H_ */
