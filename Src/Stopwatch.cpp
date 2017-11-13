/*
 * Stopwatch.cpp
 *
 *  Created on: 14. nov 2017
 *      Author: Den
 */

#include <Stopwatch.h>

Stopwatch::Stopwatch() {
  this->reset();
}

bool Stopwatch::stop() {
  #if ENABLED(DEBUG_STOPWATCH)
    Stopwatch::debug(PSTR("stop"));
  #endif

  if (this->isRunning() || this->isPaused()) {
    this->state = STOPPED;
    this->stopTimestamp = millis();
    return true;
  }
  else return false;
}

