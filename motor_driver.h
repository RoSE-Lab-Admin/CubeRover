#ifndef MOTOR
#define MOTOR

#include <RoboClaw.h>
#include <elapsedMillis.h>

void set_motor_speed(int motorIndex, uint32_t speed);
void set_motor_speeds(uint32_t lSpeed, uint32_t rSpeed);
String get_telemetry();
void encoder_reset();
void init_motor_controllers(RoboClaw* RC1, RoboClaw* RC2);
void pid_set(int arg1, int arg2, int arg3);
void safety_check(int setpoint, int v);

class Wheel {
  public:
    Wheel();
    uint16_t calcAccel(int16_t newVel);
    int16_t velocity();
  private:
    uint64_t _last;
    int16_t _prevVel;
};

#endif