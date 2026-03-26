#ifndef MOTOR
#define MOTOR

#include <EEPROM.h>
#include <elapsedMillis.h>
#include <RoboClaw.h>
#include "motor_timer.h"

void set_motor_speed(int32_t motorIndex, int32_t speed);
void set_motor_speeds(int32_t lSpeed, int32_t rSpeed);
String get_telemetry();
void encoder_reset();
void init_motor_controllers(RoboClaw* RC1, RoboClaw* RC2);
void pid_set(int32_t arg1, int32_t arg2, int32_t arg3);
void safety_check(int32_t setpoint, int32_t actual_vel, MotorTimer &motor_timer);

class Wheel {
  public:
    Wheel();
    int32_t calcAccel(int32_t newVel);
    int32_t velocity();
  private:
    uint64_t _last;
    int32_t _prevVel;
};

#endif