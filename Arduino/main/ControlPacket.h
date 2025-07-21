#ifndef CONTROL_PACKET_H
#define CONTROL_PACKET_H

//#define DEBUG

#include <RoboClaw.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <elapsedMillis.h>

elapsedMillis univTimer = 0;


class ControlPacket { // Basic controlpacket parent class
  public:
    ControlPacket() {}; // initializer -> does nothing
    virtual ~ControlPacket(); // destructor -> deletes _dataArr
    virtual void resolve(RoboClaw * RC1, RoboClaw * RC2) = 0; // resolve function -> resolves the packet
    virtual bool fulfilled(RingBuf<ControlPacket*, 20>& packetBuff) = 0;
    virtual void stop() = 0;
  protected:
    RoboClaw * _RC1;
    RoboClaw * _RC2;
    elapsedMillis _packTimer;
};

// ControlPacket Implementations:

class VelPID : public ControlPacket {
 public:
    VelPID(int * data);
    void resolve(RoboClaw * RC1, RoboClaw * RC2) final;
    bool fulfilled(RingBuf<ControlPacket*, 20>& packetBuff) final;
    void stop() final;
  private:
    int _accel;
    int _deaccel;
    int _vL1;
    int _vL2;
    int _vR1;
    int _vR2;
    long unsigned int _time;
};

class PosPID : public ControlPacket {
 public:
    PosPID(int * data);
    void resolve(RoboClaw * RC1, RoboClaw * RC2) final;
    bool fulfilled(RingBuf<ControlPacket*, 20>& packetBuff  ) final;
    void stop() final;
  private:
    int _accel;
    int _deaccel;
    int _dist;
    int _vel;
    uint8_t depth1,depth2,depth3,depth4; // for holding buffer states
};



// IMPLEMENTATIONS



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation for parent class ControlPacket
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



ControlPacket::~ControlPacket() {
  
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementations for child class VelPID (for velocity speed PID control)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



VelPID::VelPID(int * data) { // initilizes the Velocity PID speed control packet
  // grabs the data from the inputted array
  _vL1 = data[0];
  _vL2 = data[1];
  _vR1 = data[2];
  _vR2 = data[3];
  _time = data[4];
  //Serial.println("got vals");

  // saves the acceleration and deacceleration values from initialization
  _accel = data[5];
  _deaccel = data[6];
  //Serial.println("got vals2");

  // deletes the data array to prevent a memory leak
  delete[] data;
  //Serial.print("deleted array");
  digitalWrite(13,LOW);
  //Serial.print("init");
}

void VelPID::resolve(RoboClaw * RC1, RoboClaw * RC2) { // sets all motors to go at speed denoted by the only number in data
  // saves roboclaw pointers for use in entire class
  _RC1 = RC1;
  _RC2 = RC2;

  // status conditions to make sure recieved packet is valid
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  int32_t speedL1 = _RC1->ReadSpeedM1(0x80, &status1, &valid1);
  int32_t speedL2 = _RC1->ReadSpeedM2(0x80, &status2, &valid2);
  int32_t speedR1 = _RC2->ReadSpeedM1(0x80, &status3, &valid3);
  int32_t speedR2 = _RC2->ReadSpeedM2(0x80, &status4, &valid4);

  int counter = 0;

  while(!valid1 && !valid2 && !valid3 && !valid4) { // if invalid reading, read until valid encoder setpoint
    speedL1 = _RC1->ReadSpeedM1(0x80, &status1, &valid1);
    speedL2 = _RC1->ReadSpeedM2(0x80, &status2, &valid2);
    speedR1 = _RC2->ReadSpeedM1(0x80, &status3, &valid3);
    speedR2 = _RC2->ReadSpeedM2(0x80, &status4, &valid4);
    //Serial.print("oops im stuck");
    counter++;
    if (counter > 6) {
      valid1 = 1;
      valid2 = 1;
      valid3 = 1;
      valid4 = 1;
      speedL1 = _vL1;
      speedL2 = _vL2;
      speedR1 = _vR1;
      speedR2 = _vR2;
    }
  }

  // commanding l1
  bool speedingUp = (_vL1 - speedL1) * speedL1 > 0;
  int accelToUse = speedingUp ? _accel : _deaccel;
  _RC1->SpeedAccelM1(0x80, accelToUse, _vL1);
  
  // commanding l2
  speedingUp = (_vL2 - speedL2) * speedL2 > 0;
  accelToUse = speedingUp ? _accel : _deaccel;
  _RC1->SpeedAccelM2(0x80, accelToUse, _vL2);

  // commanding r1
  speedingUp = (_vR1 - speedR1) * speedR1 > 0;
  accelToUse = speedingUp ? _accel : _deaccel;
  _RC2->SpeedAccelM1(0x80, accelToUse, _vR1);
  
  // commanding r2
  speedingUp = (_vR2 - speedR2) * speedR2 > 0;
  accelToUse = speedingUp ? _accel : _deaccel;
  _RC2->SpeedAccelM2(0x80, accelToUse, _vR2);

  // set run timer to 0
  _packTimer = 0;

  //debug prints if necessary
  #ifdef DEBUG
    Serial.print("VL: ");
    Serial.print(speedL1);
    Serial.print(" | VR: ");
    Serial.print(speedR1);

    Serial.print(" | VL: ");
    Serial.print(_vL1);
    Serial.print(" | VR: ");
    Serial.print(_vR1);

    Serial.print(" | Accel: ");
    Serial.println(accelToUse);
  #endif
}

bool VelPID::fulfilled(RingBuf<ControlPacket*, 20>& packetBuff) {    // checks if packet is complete (based on placeholder 2 second timer)

  #ifdef DEBUG
    uint8_t status1,status2;
    bool valid1,valid2;
    int32_t speed1 = _RC1->ReadSpeedM1(0x80, &status1, &valid1);
    int32_t speed2 = _RC2->ReadSpeedM1(0x80, &status2, &valid2);
    Serial.print("VL: ");
    Serial.print(speed1);
    Serial.print(" | VR: ");
    Serial.println(speed2);
  #endif

  // if been running for longer than desired time
  if (_packTimer >= _time) {
    // turn off roboclaws
    this->stop();
    return true;
  } else if ( packetBuff.size() >= 1 ) {
    // if another velocity command waiting
    return true;
  } else {
    return false;
  }
}

void VelPID::stop() { // sets the roboclaws to stop
  // _RC1->SpeedAccelM1(0x80, 1000, 0);
  // _RC1->SpeedAccelM2(0x80, 1000, 0);
  // _RC2->SpeedAccelM1(0x80, 1000, 0);
  // _RC2->SpeedAccelM2(0x80, 1000, 0);
  _RC1->SpeedAccelM1M2(0x80, _deaccel, 0, 0);
  _RC2->SpeedAccelM1M2(0x80, _deaccel, 0, 0);
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementations for child class PosPID (for positional PID control)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


PosPID::PosPID(int * data) { // initilizes the POSPID Class, for position control
  // grabs the data from the inputted array
  _dist = data[0];
  _vel = data[1];

  // saves the acceleration and deacceleration values from initialization
  _accel = 0;
  _deaccel = 0;

  // deletes the data array to prevent a memory leak
  delete[] data;
  #ifdef DEBUG
    Serial.write("Distance created");
  #endif
}

// TODO -> convert to qpps
void PosPID::resolve(RoboClaw * RC1, RoboClaw * RC2) { // sends the motor commands to each roboclaw
  _RC1 = RC1;
  _RC2 = RC2;
  _RC1->ResetEncoders(0x80);
  _RC2->ResetEncoders(0x80);
  delay(50);
  _RC1->SpeedAccelDeccelPositionM1(0x80, _accel, _vel, _deaccel, _dist, 1);
  _RC1->SpeedAccelDeccelPositionM2(0x80, _accel, _vel, _deaccel, _dist, 1);
  _RC2->SpeedAccelDeccelPositionM1(0x80, _accel, _vel, _deaccel, _dist, 1);
  _RC2->SpeedAccelDeccelPositionM2(0x80, _accel, _vel, _deaccel, _dist, 1);
}

bool PosPID::fulfilled(RingBuf<ControlPacket*, 20>& packetBuff) {    // checks if packet is complete (based on placeholder 2 second timer)

  _RC1->ReadBuffers(0x80,depth1,depth2);
  _RC2->ReadBuffers(0x80,depth3,depth4);
  delay(20);

  #ifdef DEBUG
    Serial.print(depth1); Serial.print(" | ");
    Serial.print(depth2); Serial.print(" | ");
    Serial.print(depth3); Serial.print(" | ");
    Serial.print(depth4); Serial.println(" | ");
  #endif


  if (depth1==0x80 && depth2==0x80 && depth3==0x80 && depth4==0x80) { // if all 4 roboclaw buffers state POS is reached
    return true;
  } else {
    return false;
  }
}

void PosPID::stop() { // sets the roboclaws to stop before the packet is fulfilled
  _RC1->SpeedAccelM1M2(0x80, _deaccel, 0, 0);
  _RC2->SpeedAccelM1M2(0x80, _deaccel, 0, 0);
}





#endif

