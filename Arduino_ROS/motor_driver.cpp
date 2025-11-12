#include <RoboClaw.h>
#include <EEPROM.h>
#include "motor_driver.h"

#define qpps 3400     // quadrature pulses per second at max rpm
#define ADDRESS 0x80  // default roboclaw address - 128



// pointers to store reference to roboclaws on init
extern RoboClaw *ROBOCLAW_1;
extern RoboClaw *ROBOCLAW_2;


// define wheel classes to store relevant wheel info
Wheel FL;
Wheel BL;
Wheel FR;
Wheel BR;


// start custom function implementations
void set_motor_speed(int motorIndex, uint32_t speed) {
  if (motorIndex == 1)      ROBOCLAW_1->SpeedAccelM1(0x80, FL.calcAccel(speed), speed);
  else if (motorIndex == 2) ROBOCLAW_1->SpeedAccelM2(0x80, BL.calcAccel(speed), speed);
  else if (motorIndex == 3) ROBOCLAW_2->SpeedAccelM1(0x80, FR.calcAccel(speed), speed);
  else if (motorIndex == 4) ROBOCLAW_2->SpeedAccelM2(0x80, BR.calcAccel(speed), speed);
}


void set_motor_speeds(uint32_t lSpeed, uint32_t rSpeed) {

  ROBOCLAW_1->SpeedM1M2(0x80, lSpeed, lSpeed);
  ROBOCLAW_2->SpeedM1M2(0x80, rSpeed, rSpeed);

  // ROBOCLAW_1->SpeedAccelM1M2(0x80, FL.calcAccel(lSpeed), lSpeed, lSpeed);
  // ROBOCLAW_1->SpeedAccelM1M2(0x80, FR.calcAccel(rSpeed), rSpeed, rSpeed);
  //Serial.println(BL.calcAccel(lSpeed)); // print out accel
  FL.calcAccel(lSpeed);
  FR.calcAccel(rSpeed);
  BL.calcAccel(lSpeed);
  BR.calcAccel(rSpeed);
  // ROBOCLAW_1->SpeedAccelM1(0x80, FL.calcAccel(lSpeed), lSpeed);
  // ROBOCLAW_1->SpeedAccelM2(0x80, BL.calcAccel(lSpeed), lSpeed);
  // ROBOCLAW_2->SpeedAccelM1(0x80, FR.calcAccel(rSpeed), rSpeed); 
  // ROBOCLAW_2->SpeedAccelM2(0x80, BR.calcAccel(rSpeed), rSpeed);

  return;
}


String get_telemetry() {
  // uint32_t start = millis();
  int32_t telemetryData[14];

  // Retrieve Encoder counts
  uint32_t count1=0, count2=0, count3=0, count4=0;
  for (int i = 0; i < 3; i ++) {
    bool v1,v2; //v3,v4;
    // uint8_t s1,s2,s3,s4;
    // count1 = ROBOCLAW_1->ReadEncM1(0x80, &s1, &v1);
    // count2 = ROBOCLAW_1->ReadEncM2(0x80, &s2, &v2);
    // count3 = ROBOCLAW_2->ReadEncM1(0x80, &s3, &v3);
    // count4 = ROBOCLAW_2->ReadEncM2(0x80, &s4, &v4);
    v1 = ROBOCLAW_1->ReadEncoders(0x80, count1, count2);
    v2 = ROBOCLAW_2->ReadEncoders(0x80, count3, count4);
    if (v1 && v2) break;
  }

  telemetryData[0] = (int32_t)count1;
  telemetryData[1] = (int32_t)count2;
  telemetryData[2] = (int32_t)count3;
  telemetryData[3] = (int32_t)count4;


  // Retrieve Encoder velocities
  uint32_t speed1=0, speed2=0, speed3=0, speed4=0;
  for (int i = 0; i < 3; i++) {
    uint8_t status5,status6,status7,status8;
    bool v1, v2, v3, v4;
    speed1 = ROBOCLAW_1->ReadSpeedM1(0x80, &status5, &v1);
    speed2 = ROBOCLAW_1->ReadSpeedM2(0x80, &status6, &v2);
    speed3 = ROBOCLAW_2->ReadSpeedM1(0x80, &status7, &v3);
    speed4 = ROBOCLAW_2->ReadSpeedM2(0x80, &status8, &v4);
    if (v1 && v2 && v3 && v4) break;
  }

  safety_check(FL.velocity(), speed1);
  safety_check(BL.velocity(), speed2);
  safety_check(FR.velocity(), speed3);
  safety_check(BR.velocity(), speed4);

  telemetryData[4] = (int32_t)speed1;
  telemetryData[5] = (int32_t)speed2;
  telemetryData[6] = (int32_t)speed3;
  telemetryData[7] = (int32_t)speed4;


  // Read Currents
  int16_t c1, c2, c3, c4;
  for (size_t i = 0; i < 3; i++) {
    bool rc1cval, rc2cval;
    rc1cval = ROBOCLAW_1->ReadCurrents(0x80, c1, c2);
    rc2cval = ROBOCLAW_2->ReadCurrents(0x80, c3, c4);
    if (rc1cval && rc2cval) break;
  }
  telemetryData[8] = (int32_t)c1;
  telemetryData[9] = (int32_t)c2;
  telemetryData[10] = (int32_t)c3;
  telemetryData[11] = (int32_t)c4;

  // Read battery voltages
  uint16_t v1 = 0, v2 = 0;
  for (size_t i = 0; i < 3; i++) {
    bool v1val, v2val;
    v1 = ROBOCLAW_1->ReadMainBatteryVoltage(0x80, &v1val);
    v2 = ROBOCLAW_2->ReadMainBatteryVoltage(0x80, &v2val);
    if (v1val && v2val) break;
  }

  telemetryData[12] = (int32_t)v1;
  telemetryData[13] = (int32_t)v2;


  //build return telemetry carrige
  String telemetry;
  telemetry.reserve(64);
  telemetry += 'e';
  for (size_t i = 0; i < 14; i++) telemetry += ' ' + String(telemetryData[i]);
  telemetry += "\r\n";

  // uint32_t dur = millis() - start;
  // Serial.println(dur);

  return telemetry;
}


void encoder_reset() {
  ROBOCLAW_1->ResetEncoders(0x80);
  ROBOCLAW_2->ResetEncoders(0x80);
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
  ROBOCLAW_1->SetM1VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps); // change the velocity settings
  ROBOCLAW_1->SetM2VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps);
  ROBOCLAW_2->SetM1VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps);
  ROBOCLAW_2->SetM2VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps);
  //Serial.println("Motor PID set");
}


void safety_check(int setpoint, int v) {
  if (!(setpoint * v >= 0) && setpoint != 0) {
    Serial.println("ENCODDER ERROR! CHECK WIRE!");
    set_motor_speeds(0, 0);
    while (true) {
      digitalWrite(13,HIGH);
      delay(500);
      digitalWrite(13,LOW);
      delay(500);
      Serial.println("ENCODDER ERROR! CHECK WIRE!");
    }
  }

  if (abs(v) > qpps * 0.75) {
    Serial.println("VELOCITY SETPOINT ERROR!");
    set_motor_speeds(0, 0);
    while (true) {
      digitalWrite(13,HIGH);
      delay(1000);
      digitalWrite(13,LOW);
      delay(1000);
      Serial.println("VELOCITY SETPOINT ERROR!");
    }
  }
}



// start wheel class implementations
Wheel::Wheel() {
  _prevVel = 0;
  _last = micros();
  return;
}


uint16_t Wheel::calcAccel(int16_t newVel){
  uint64_t now = micros();
  double dt = (now - _last) / 1e6f;
  if (dt <= 0) dt = 1e-6;
  uint16_t target_acl = abs(newVel - _prevVel) / dt;
  //Serial.println(target_acl);
  _prevVel = newVel;
  _last = now;
  return target_acl;
 }  

int16_t Wheel::velocity(){
  return _prevVel;
}
