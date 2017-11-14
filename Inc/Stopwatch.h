/*
 * Stopwatch.h
 *
 *  Created on: 14. nov 2017
 *      Author: Den
 */

#ifndef STOPWATCH_H_
#define STOPWATCH_H_

class Stopwatch {
  private:
    enum State {
      STOPPED,
      RUNNING,
      PAUSED
    };

    Stopwatch::State state;
    millis_t stopTimestamp;
    millis_t startTimestamp;
    millis_t accumulator;

  public:
	Stopwatch();

    /**
     * @brief Stops the stopwatch
     * @details Stops the running timer, it will silently ignore the request if
     * no timer is currently running.
     * @return true is method was successful
     */
    bool stop();

    /**
     * @brief Resets the stopwatch
     * @details Resets all settings to their default values.
     */
    void reset();

    /**
     * @brief Checks if the timer is running
     * @details Returns true if the timer is currently running, false otherwise.
     * @return true if stopwatch is running
     */
    bool isRunning();

    /**
     * @brief Checks if the timer is paused
     * @details Returns true if the timer is currently paused, false otherwise.
     * @return true if stopwatch is paused
     */
    bool isPaused();

};

#endif /* STOPWATCH_H_ */
