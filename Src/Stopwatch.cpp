/*
 * Stopwatch.cpp
 *
 *  Created on: 14. nov 2017
 *      Author: Den
 */

#include "main.h"
#include "Stopwatch.h"

Stopwatch::Stopwatch() {
  this->reset();
}

bool Stopwatch::stop() {
  if (this->isRunning() || this->isPaused()) {
    this->state = STOPPED;
    this->stopTimestamp = millis();
    return true;
  }
  else return false;
}

void Stopwatch::reset() {
  this->state = STOPPED;
  this->startTimestamp = 0;
  this->stopTimestamp = 0;
  this->accumulator = 0;
}

bool Stopwatch::isRunning() {
  return (this->state == RUNNING) ? true : false;
}

bool Stopwatch::isPaused() {
  return (this->state == PAUSED) ? true : false;
}

