#ifndef MOTOR
#define MOTOR

#include <EEPROM.h>
#include <elapsedMillis.h>
#include <RoboClaw.h>
#include "messages.h"
#include "motor_timer.h"

void set_motor_speed(int motorIndex, uint32_t speed);
void set_motor_speeds(uint32_t lSpeed, uint32_t rSpeed);
String get_telemetry();
void encoder_reset();
void init_motor_controllers(RoboClaw* RC1, RoboClaw* RC2);
void pid_set(int arg1, int arg2, int arg3);
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