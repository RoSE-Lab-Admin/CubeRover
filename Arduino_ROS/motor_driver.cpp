#include "motor_driver.h"

#define MAX_QPPS 3400     // quadrature pulses per second at max rpm
#define ADDRESS 0x80  // default roboclaw address - 128


// pointers to store reference to roboclaws on init
extern RoboClaw *ROBOCLAW_1;
extern RoboClaw *ROBOCLAW_2;


// define wheel classes to store relevant wheel info
Wheel FL;
Wheel BL;
Wheel FR;
Wheel BR;

MotorTimer FL_timer;
MotorTimer BL_timer;
MotorTimer FR_timer;
MotorTimer BR_timer;

// At the top of your file
const char* FL_name = "front left";
const char* BL_name = "back left";
const char* FR_name = "front right";
const char* BR_name = "back right";

// Hidden internal state variables
static bool _is_faulted = false;
static MessageCode _current_error_code;
static String _current_error_message;
static uint32_t _blink_interval = 500;
static uint32_t _last_blink_time = 0;
static bool _led_state = false;


// start custom function implementations
void set_motor_speed(int motorIndex, uint32_t speed) {
  if (motorIndex == 1)      ROBOCLAW_1->SpeedAccelM1(ADDRESS, FL.calcAccel(speed), speed);
  else if (motorIndex == 2) ROBOCLAW_1->SpeedAccelM2(ADDRESS, BL.calcAccel(speed), speed);
  else if (motorIndex == 3) ROBOCLAW_2->SpeedAccelM1(ADDRESS, FR.calcAccel(speed), speed);
  else if (motorIndex == 4) ROBOCLAW_2->SpeedAccelM2(ADDRESS, BR.calcAccel(speed), speed);
}


void set_motor_speeds(uint32_t lSpeed, uint32_t rSpeed) {

  ROBOCLAW_1->SpeedM1M2(ADDRESS, lSpeed, lSpeed);
  ROBOCLAW_2->SpeedM1M2(ADDRESS, rSpeed, rSpeed);

  // ROBOCLAW_1->SpeedAccelM1M2(ADDRESS, FL.calcAccel(lSpeed), lSpeed, lSpeed);
  // ROBOCLAW_1->SpeedAccelM1M2(ADDRESS, FR.calcAccel(rSpeed), rSpeed, rSpeed);
  //Serial.println(BL.calcAccel(lSpeed)); // print out accel
  FL.calcAccel(lSpeed);
  FR.calcAccel(rSpeed);
  BL.calcAccel(lSpeed);
  BR.calcAccel(rSpeed);
  // ROBOCLAW_1->SpeedAccelM1(ADDRESS, FL.calcAccel(lSpeed), lSpeed);
  // ROBOCLAW_1->SpeedAccelM2(ADDRESS, BL.calcAccel(lSpeed), lSpeed);
  // ROBOCLAW_2->SpeedAccelM1(ADDRESS, FR.calcAccel(rSpeed), rSpeed); 
  // ROBOCLAW_2->SpeedAccelM2(ADDRESS, BR.calcAccel(rSpeed), rSpeed);

  return;
}


String get_telemetry() {
  const uint16_t TELEMETRY_DATA_SIZE = 18; // IMPORTANT: If this changes, search for this variable name in other files and update there too!!
  const uint16_t CAPTURE_ATTEMPTS = 3;

  // uint32_t start = millis();

  // Zero-initialize the entire array. If a sensor fails all attempts, 
  // it safely reports '0' instead of random memory garbage.
  int32_t telemetryData[TELEMETRY_DATA_SIZE] = {0};

  // Retrieve Encoder counts
  uint32_t count1 = 0, count2 = 0, count3 = 0, count4 = 0;
  for (int i = 0; i < CAPTURE_ATTEMPTS; i++) {
      bool v1, v2;
      v1 = ROBOCLAW_1->ReadEncoders(ADDRESS, count1, count2);
      v2 = ROBOCLAW_2->ReadEncoders(ADDRESS, count3, count4);
      if (v1 && v2) break;
  }

  // Note: These are safe typecasts since the encoder count is actually signed.
  // See manual "16 - Read Encoder Count/Value M1" (encoder values can have a direction).
  telemetryData[0] = (int32_t) count1;
  telemetryData[1] = (int32_t) count2;
  telemetryData[2] = (int32_t) count3;
  telemetryData[3] = (int32_t) count4;


  // Retrieve Encoder velocities
  uint32_t speed1 = 0, speed2 = 0, speed3 = 0, speed4 = 0;
  for (int i = 0; i < CAPTURE_ATTEMPTS; i++) {
    uint8_t status5, status6, status7, status8;
    bool v1, v2, v3, v4;
    speed1 = ROBOCLAW_1->ReadSpeedM1(ADDRESS, &status5, &v1);
    speed2 = ROBOCLAW_1->ReadSpeedM2(ADDRESS, &status6, &v2);
    speed3 = ROBOCLAW_2->ReadSpeedM1(ADDRESS, &status7, &v3);
    speed4 = ROBOCLAW_2->ReadSpeedM2(ADDRESS, &status8, &v4);
    if (v1 && v2 && v3 && v4) break;
  }

  // Note: These are safe typecasts since the speed is actually signed.
  // See manual "18 - Read Encoder Speed M1" (speeds have direction)
  // Also see manual "35 - Drive M1 With Signed Speed" (input speeds are signed)
  int32_t FL_speed = (int32_t) speed1;
  int32_t BL_speed = (int32_t) speed2;
  int32_t FR_speed = (int32_t) speed3;
  int32_t BR_speed = (int32_t) speed4;

  safety_check(FL.velocity(), FL_speed, FL_timer, FL_name);
  safety_check(BL.velocity(), BL_speed, BL_timer, BL_name);
  safety_check(FR.velocity(), FR_speed, FR_timer, FR_name);
  safety_check(BR.velocity(), BR_speed, BR_timer, BR_name);

  telemetryData[4] = FL_speed;
  telemetryData[5] = BL_speed;
  telemetryData[6] = FR_speed;
  telemetryData[7] = BR_speed;


  // Read Currents
  int16_t c1 = 0, c2 = 0, c3 = 0, c4 = 0;
  for (int i = 0; i < CAPTURE_ATTEMPTS; i++) {
    bool rc1cval, rc2cval;
    rc1cval = ROBOCLAW_1->ReadCurrents(ADDRESS, c1, c2);
    rc2cval = ROBOCLAW_2->ReadCurrents(ADDRESS, c3, c4);
    if (rc1cval && rc2cval) break;
  }
  telemetryData[8] = (int32_t) c1;
  telemetryData[9] = (int32_t) c2;
  telemetryData[10] = (int32_t) c3;
  telemetryData[11] = (int32_t) c4;


  // Read battery voltages
  uint16_t v1 = 0, v2 = 0;
  for (int i = 0; i < CAPTURE_ATTEMPTS; i++) {
    bool v1val, v2val;
    v1 = ROBOCLAW_1->ReadMainBatteryVoltage(ADDRESS, &v1val);
    v2 = ROBOCLAW_2->ReadMainBatteryVoltage(ADDRESS, &v2val);
    if (v1val && v2val) break;
  }
  telemetryData[12] = (int32_t) v1;
  telemetryData[13] = (int32_t) v2;


  // Read PWM values
  int16_t pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;
  for (int i = 0; i < CAPTURE_ATTEMPTS; i++) {
    bool pwms1val, pwms2val;
    pwms1val = ROBOCLAW_1->ReadPWMs(ADDRESS, pwm1, pwm2);
    pwms2val = ROBOCLAW_2->ReadPWMs(ADDRESS, pwm3, pwm4);
    if (pwms1val && pwms2val) break;
  }
  telemetryData[14] = (int32_t) pwm1;
  telemetryData[15] = (int32_t) pwm2;
  telemetryData[16] = (int32_t) pwm3;
  telemetryData[17] = (int32_t) pwm4;


  // Build return telemetry string
  String telemetry;
  telemetry.reserve(256); // 32-bit signed integer can take up to 11 chars + 1 space = 12 per number.
  telemetry += TELEMETRY_MESSAGE;
  for (size_t i = 0; i < TELEMETRY_DATA_SIZE; i++) {
    // Note: Arduino String library has overloads to handle directly appending int32_t to String
    // The space and data MUST be added individually for the compiler to recognize these are two differnet pieces of data
    telemetry += ' ';
    telemetry += telemetryData[i];
  }
  telemetry += "\r\n";

  // uint32_t dur = millis() - start;
  // Serial.println(dur);

  return telemetry;
}


void encoder_reset() {
  ROBOCLAW_1->ResetEncoders(ADDRESS);
  ROBOCLAW_2->ResetEncoders(ADDRESS);
}


void pid_set(int arg1, int arg2, int arg3) {
  float p = static_cast<float>(arg1) / 100;
  float i = static_cast<float>(arg2) / 100;
  float d = static_cast<float>(arg3) / 100;
  EEPROM.put(0, p);
  EEPROM.put(4, i);
  EEPROM.put(8, d);
}


void init_motor_controllers(RoboClaw* RC1, RoboClaw* RC2) {
  ROBOCLAW_1 = RC1;
  ROBOCLAW_2 = RC2;
  float fsettings[3] = {0}; // stores float settings in an array. [vP,vI,vD]
  for (size_t idx = 0; idx < 3; idx++) {
    EEPROM.get(idx * sizeof(float), fsettings[idx]);
  }
  ROBOCLAW_1->SetM1VelocityPID(ADDRESS, fsettings[0], fsettings[1], fsettings[2], MAX_QPPS); // change the velocity settings
  ROBOCLAW_1->SetM2VelocityPID(ADDRESS, fsettings[0], fsettings[1], fsettings[2], MAX_QPPS);
  ROBOCLAW_2->SetM1VelocityPID(ADDRESS, fsettings[0], fsettings[1], fsettings[2], MAX_QPPS);
  ROBOCLAW_2->SetM2VelocityPID(ADDRESS, fsettings[0], fsettings[1], fsettings[2], MAX_QPPS);
  //Serial.println("Motor PID set");
}


bool is_system_faulted() {
  return _is_faulted;
}


void clear_system_fault() {
  _is_faulted = false;
  digitalWrite(13, HIGH); // Set LED back to solid on
}


void update_fault_led() {
  if (!_is_faulted) return;
  
  uint32_t current_time = millis();
  if (current_time - _last_blink_time >= _blink_interval) {
    _last_blink_time = current_time;
    _led_state = !_led_state;
    digitalWrite(13, _led_state ? HIGH : LOW);
  }
}


void enter_error_state(MessageCode msg_code, const String& message, uint32_t blink_interval_ms) {
  set_motor_speeds(0, 0); 
  send_message(msg_code, message);

  _is_faulted = true;
  _current_error_code = msg_code;
  _current_error_message = message;
  _blink_interval = blink_interval_ms;
  _last_blink_time = millis();
}


void safety_check(int32_t setpoint, int32_t actual_vel, MotorTimer &motor_timer, const char* motor_name) {
  // If the system is already faulted, don't keep triggering new faults
  if (is_system_faulted()) {
    return;
  }

  const int32_t NOISE_FLOOR_PERCENT = 2;
  const int32_t NOISE_FLOOR_QPPS = (NOISE_FLOOR_PERCENT * MAX_QPPS) / 100;
  const uint32_t OPPOSITE_DIR_THRESHOLD_MS = 500; // Half a second of consistently moving in the wrong direction

  // Check whether signs are opposite.
  bool is_opposite = (setpoint > 0 && actual_vel < 0) || (setpoint < 0 && actual_vel > 0);

  // Is it actually moving, or is it just sensor noise?
  bool is_physically_moving = abs(actual_vel) > NOISE_FLOOR_QPPS;
  if (is_opposite && is_physically_moving) {
    motor_timer.start();

    // The fault is active! Check whether time threshold exceeded.
    if (motor_timer.hasExpired(OPPOSITE_DIR_THRESHOLD_MS)) {
      String message = "Check ";
      message += motor_name;
      message += " motor wires!";

      // Trigger the 500ms blink loop
      enter_error_state(MessageCode::CHECK_ENCODER, message, 500);

      // Prevent instant re-trigger when the loop exits
      motor_timer.reset();
      return;
    }
  } else {
    // Motor is behaving correctly (or just experiencing tiny noise), so reset the timer
    motor_timer.reset();
  }

  // Max allowable velocity threshold
  const int32_t MAX_QPPS_PERCENT = 75;
  const int32_t MAX_VEL_QPPS = (MAX_QPPS_PERCENT * MAX_QPPS) / 100;
  if (abs(actual_vel) > MAX_VEL_QPPS) {
    String message = "Velocity setpoint error on ";
    message += motor_name;
    message += " motor!";

    // Trigger the 1000ms blink loop
    enter_error_state(MessageCode::CHECK_VELOCITY, message, 1000);
    return;
  }
}



// start wheel class implementations
Wheel::Wheel() {
  _prevVel = 0;
  _last = micros();
  return;
}


int32_t Wheel::calcAccel(int32_t newVel){
  uint64_t now = micros();
  double dt = (now - _last) / 1e6f;
  if (dt <= 0) dt = 1e-6;
  int32_t target_acl = abs(newVel - _prevVel) / dt;
  //Serial.println(target_acl);
  _prevVel = newVel;
  _last = now;
  return target_acl;
 }  

int32_t Wheel::velocity(){
  return _prevVel;
}
