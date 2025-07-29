#ifndef MOTOR
#define MOTOR

#include <RoboClaw.h>

extern RoboClaw* ROBOCLAW_1;
extern RoboClaw* ROBOCLAW_2;

void set_motor_speed(int motorIndex, int speed, int accel);
void set_motor_speeds(int lSpeed, int rSpeed, int accel);
String get_telemetry();
void encoder_reset();
void init_motor_controllers(RoboClaw* RC1, RoboClaw* RC2);

#endif