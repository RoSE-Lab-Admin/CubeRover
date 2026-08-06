#ifndef MOTOR_TIMER
#define MOTOR_TIMER

#include <elapsedMillis.h>

class MotorTimer {
  private:
    elapsedMillis timer;
    bool is_running;

  public:
    MotorTimer();

    // Starts the timer from 0, but ONLY if it isn't already running
    void start();

    // Stops and resets the timer
    void reset();

    // Checks if the timer is running AND has passed the threshold
    bool hasExpired(uint32_t threshold_ms);
};

#endif