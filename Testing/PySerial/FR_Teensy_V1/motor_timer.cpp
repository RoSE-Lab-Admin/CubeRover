#include "motor_timer.h"

MotorTimer::MotorTimer() {
    is_running = false;
}

void MotorTimer::start() {
    if (!is_running) {
        timer = 0;
        is_running = true;
    }
}

void MotorTimer::reset() {
    is_running = false;
}

bool MotorTimer::hasExpired(uint32_t threshold_ms) {
    return is_running && (timer >= threshold_ms);
}
