#ifndef MOTOR
#define SERVO

#include <Arduino.h>
#include <Servo.h>
#include "servo.h"

// Constructor
SweepServo::SweepServo()
{
  this->currentPositionDegrees = 0;
  this->targetPositionDegrees = 0;
  this->lastSweepCommand = 0;
}


// Init
void SweepServo::initServo(
    int servoPin,
    int stepDelayMs,
    int initPosition)
{
  this->servo.attach(servoPin);
  this->stepDelayMs = stepDelayMs;
  this->currentPositionDegrees = initPosition;
  this->targetPositionDegrees = initPosition;
  this->lastSweepCommand = millis();
}


// Perform Sweep
void SweepServo::doSweep()
{

  // Get ellapsed time
  int delta = millis() - this->lastSweepCommand;

  // Check if time for a step
  if (delta > this->stepDelayMs) {
    // Check step direction
    if (this->targetPositionDegrees > this->currentPositionDegrees) {
      this->currentPositionDegrees++;
      this->servo.write(this->currentPositionDegrees);
    }
    else if (this->targetPositionDegrees < this->currentPositionDegrees) {
      this->currentPositionDegrees--;
      this->servo.write(this->currentPositionDegrees);
    }
    // if target == current position, do nothing
    //Serial.println(this->currentPositionDegrees);
    // reset timer
    this->lastSweepCommand = millis();
  }
}


// Set a new target position
void SweepServo::setTargetPosition(int position)
{
  this->targetPositionDegrees = position;
}


// Accessor for servo object
Servo SweepServo::getServo()
{
  return this->servo;
}



#endif