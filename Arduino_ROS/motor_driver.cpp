#include <RoboClaw.h>
#include <EEPROM.h>

#define qpps 3400     // quadrature pulses per second at max rpm
#define ADDRESS 0x80  // default roboclaw address - 128

// pointers to store reference to roboclaws on init
extern RoboClaw *ROBOCLAW_1;
extern RoboClaw *ROBOCLAW_2;


void set_motor_speed(int motorIndex, int speed, int accel) {
  if (motorIndex == 1)      ROBOCLAW_1->SpeedAccelM1(0x80, accel, speed);
  else if (motorIndex == 2) ROBOCLAW_1->SpeedAccelM2(0x80, accel, speed);
  else if (motorIndex == 3) ROBOCLAW_2->SpeedAccelM1(0x80, accel, speed);
  else if (motorIndex == 4) ROBOCLAW_2->SpeedAccelM2(0x80, accel, speed);
}


void set_motor_speeds(int lSpeed, int rSpeed, int accel) {
  ROBOCLAW_1->SpeedAccelM1(0x80, accel, lSpeed);
  ROBOCLAW_1->SpeedAccelM2(0x80, accel, lSpeed);
  ROBOCLAW_2->SpeedAccelM1(0x80, accel, rSpeed);
  ROBOCLAW_2->SpeedAccelM2(0x80, accel, rSpeed);
}


String get_telemetry() {
  int * telemetryData = new int[14];

  // Retrieve Encoder counts
  uint8_t status1,status2,status3,status4;
  bool valid1=false,valid2=false,valid3=false,valid4=false;
  uint32_t count1=0, count2=0, count3=0, count4=0;
  for (int i = 0; i < 5; i ++) {
      count1 = ROBOCLAW_1->ReadEncM1(0x80, &status1, &valid1);
      count2 = ROBOCLAW_1->ReadEncM2(0x80, &status2, &valid2);
      count3 = ROBOCLAW_2->ReadEncM1(0x80, &status3, &valid3);
      count4 = ROBOCLAW_2->ReadEncM2(0x80, &status4, &valid4);
      if (valid1 && valid2 && valid3 && valid4) break;
  }

  telemetryData[0] = (int)count1;
  telemetryData[1] = (int)count2;
  telemetryData[2] = (int)count3;
  telemetryData[3] = (int)count4;


  // Retrieve Encoder velocities
  uint8_t status5,status6,status7,status8;
  bool valid5=false, valid6=false, valid7=false, valid8=false;
  uint32_t speed1=0, speed2=0, speed3=0, speed4=0;
  for (int i = 0; i < 5; i++) {
      speed1 = ROBOCLAW_1->ReadSpeedM1(0x80, &status5, &valid5);
      speed2 = ROBOCLAW_1->ReadSpeedM2(0x80, &status6, &valid6);
      speed3 = ROBOCLAW_2->ReadSpeedM1(0x80, &status7, &valid7);
      speed4 = ROBOCLAW_2->ReadSpeedM2(0x80, &status8, &valid8);
      if (valid5 && valid6 && valid7 && valid8) break;
  }

  telemetryData[4] = (int)speed1;
  telemetryData[5] = (int)speed2;
  telemetryData[6] = (int)speed3;
  telemetryData[7] = (int)speed4;


  // Read Currents
  int16_t c1, c2, c3, c4;
  bool rc1cval = false, rc2cval=false;
  for (size_t i = 0; i < 5; i++) {
    rc1cval = ROBOCLAW_1->ReadCurrents(0x80, c1, c2);
    rc2cval = ROBOCLAW_2->ReadCurrents(0x80, c3, c4);
    if (rc1cval && rc2cval) break;
  }
  telemetryData[8] = (int)c1;
  telemetryData[9] = (int)c2;
  telemetryData[10] = (int)c3;
  telemetryData[11] = (int)c4;

  // Read battery voltages
  uint16_t v1 = 0, v2 = 0;
  bool v1val = false, v2val = false;
  for (size_t i = 0; i < 5; i++) {
    v1 = ROBOCLAW_1->ReadMainBatteryVoltage(0x80, &v1val);
    v2 = ROBOCLAW_2->ReadMainBatteryVoltage(0x80, &v2val);
    if (v1val && v2val) break;
  }

  telemetryData[12] = (int)v1;
  telemetryData[13] = (int)v2;

  String telemetry = 'e';
  for (size_t i = 0; i < 14; i++) telemetry += ' ' + String(telemetryData[i]);
  telemetry = telemetry + '\r';
  return telemetry;
}


void encoder_reset() {
  ROBOCLAW_1->ResetEncoders(0x80);
  ROBOCLAW_2->ResetEncoders(0x80);
}


void init_motor_controllers(RoboClaw* RC1, RoboClaw* RC2) {
  ROBOCLAW_1 = RC1;
  ROBOCLAW_2 = RC2;
  float fsettings[10] = {0}; // stores float settings in an array. [vP,vI,vD,pP,pI,pD,pMI,Deadzone]
  for (size_t i = 0; i < 3*4; i = i + 4) {
    EEPROM.get((i), fsettings[i/4]);
  }
  ROBOCLAW_1->SetM1VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps); // change the velocity settings
  ROBOCLAW_1->SetM2VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps);
  ROBOCLAW_2->SetM1VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps);
  ROBOCLAW_2->SetM1VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps);
}
