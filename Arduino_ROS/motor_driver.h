#ifndef MOTOR
#define MOTOR

#include <RoboClaw.h>
#include <elapsedMillis.h>

extern RoboClaw* ROBOCLAW_1;
extern RoboClaw* ROBOCLAW_2;

void set_motor_speed(int motorIndex, int speed);
void set_motor_speeds(int lSpeed, int rSpeed);
String get_telemetry();
void encoder_reset();
void init_motor_controllers(RoboClaw* RC1, RoboClaw* RC2);
void pid_set(int arg1, int arg2, int arg3);

class Wheel{
  public:
    int calcAccel(int newVel);
  private:
    elapsedMillis _dt;
    int _vel;
};

#endif