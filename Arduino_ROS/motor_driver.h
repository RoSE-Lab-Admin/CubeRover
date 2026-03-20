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
void set_safety_params(int32_t noise_floor, uint32_t opp_dir_ms, int32_t max_qpps);
void safety_check(int32_t setpoint, int32_t actual_vel, MotorTimer &motor_timer, const char* motor_name);

// These functions allow the main Arduino loop to safely interact with the motor driver's error state.
bool is_system_faulted(); // Asks if the motors are currently in an error state
void clear_system_fault(); // Resets the error state back to normal
void update_fault_led(); // Handles the LED blinking

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