
// Including packages
#include <RingBuf.h>
#include <elapsedMillis.h>
#include <SerialTransfer.h>
#include <RoboClaw.h>
#include "ControlPacket.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>



// Robot Parameters
float Kp = 11.37910;    // proportional constant for velocity PID
float Ki = 0.345;   // integral constant for velocity PID
float Kd = 0;   // derivative constant for velocity PID
float qpps = 3400; // countable quadrature pulses per second -> found using roboclaw's basicMicro tool


// Serial Transfer declaration
SerialTransfer rx;


// init roboclaw objects to their serial ports for packet communication
RoboClaw ROBOCLAW_1 = RoboClaw(&Serial1, 10000);
RoboClaw ROBOCLAW_2 = RoboClaw(&Serial3, 10000);


void setup(void) {
  // Init serial ports for PI / computer communication
  Serial.begin(115200); // Built-in USB port for Teensy
  //Serial.println("Booting...");

  // Init serial ports for roboclaws

  // turn status LED on
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  
  // Init roboclaw values from memory
  ROBOCLAW_1.begin(38400);
  ROBOCLAW_2.begin(38400);

  delay(3000);
  rx.begin(Serial);
  WaitForRoboclaw(ROBOCLAW_1);
  WaitForRoboclaw(ROBOCLAW_2);
  MemSetup(ROBOCLAW_1, ROBOCLAW_2);
  Serial.println("Complete!");
  
}


ControlPacket * control = nullptr; // control pointer variable to keep track of what command is being done
RingBuf<ControlPacket*, 20> packetBuff; // packet buffer to keep commands if one is currently being resolved
elapsedMillis sendTimer = 0;
void loop() { // Stuff to loop over
  if (control == nullptr) {   // checks if there is currently a control packet commanding the rover, if yes:
    if (!packetBuff.isEmpty()) {    // checks the packet buffer to see if there is a command in queue, if yes:
      packetBuff.pop(control);    // adds queued command to control pointer
      control->resolve(&ROBOCLAW_1, &ROBOCLAW_2);   // resolves control pointer command
    } else if (rx.available()) {    // if no commands in packetBuffer, check serial port, if yes:
      control = SerialDecode();   // adds serial buffer command to control pointer
      if (control != nullptr) {
        control->resolve(&ROBOCLAW_1, &ROBOCLAW_2);   // resolves control pointer command
      }
    }
  } 
  if (control != nullptr) { // if control packet is currently commanding rover:
    if (control->fulfilled(packetBuff)) {   // is the packet done? If yes:
      delete control;   // delete control packet
      control = nullptr;    // set control packet to nullptr
    } else {    // if not done:
      if (rx.available()) {   // check serial buffer:
        packetBuff.push(SerialDecode());    // add serial buffer command to packet buffer
      }
    }
  }

  if (sendTimer > 60) {
    SendTelem(RetrieveTelemetry(ROBOCLAW_1, ROBOCLAW_2), 14);
    sendTimer = 0;
  }
  //delay(5);
}


ControlPacket* SerialDecode () {
  
  uint16_t recievePOS = 0; // stores position of iterator in recieving buffer
  char ID; // stores ID of current packet decoder
  ControlPacket * controlTemp = nullptr; // temp pointer to decoded packet
  recievePOS = rx.rxObj(ID, recievePOS); // store ID char
  Serial.print(ID);
  if (ID == 'V') { // Velocity control
    controlTemp = new VelPID(RetrieveSerial<int>(7, recievePOS)); // creates new packet of type Velocity
  } else if (ID == 'P') { // Distance / Position control
    //Serial.write("Distance!");
    controlTemp = new PosPID(RetrieveSerial<int>(2,recievePOS));
  } else if (ID == 'E') { // stops the rover and empties the packet buffer -> ideally for emergency / resetting the rover --> FIX
    if (control != nullptr) {
      control->stop();
      packetBuff.clear();
      delete control;
      controlTemp = nullptr;
    }
  } else if (ID == 'W') { // write to EEPROM memory and resets the Roboclaw PID data
    float * dataPTR = RetrieveSerial<float>(2,recievePOS);
    MemWrite(static_cast<float>(dataPTR[0]), dataPTR[1]);
    MemSetup(ROBOCLAW_1, ROBOCLAW_2);
  } else { // Base case
    controlTemp = nullptr;
  }
  
  return controlTemp; // returns pointer to decoded packet

}

// fetch stored configuration parameters and assign to required locations
void MemSetup(RoboClaw & RC1, RoboClaw & RC2) {
  float fsettings[10] = {0}; // stores float settings in an array. [vP,vI,vD,pP,pI,pD,pMI,Deadzone]
  for (size_t i = 0; i < 8*4; i = i + 4) {
    EEPROM.get((i), fsettings[i/4]);
    Serial.println(fsettings[i/4]);
  }
  RC1.SetM1VelocityPID(0x80, Kp, Ki, Kd, qpps); // change the velocity settings
  RC1.SetM2VelocityPID(0x80, Kp, Ki, Kd, qpps);
  RC2.SetM1VelocityPID(0x80, Kp, Ki, Kd, qpps);
  RC2.SetM2VelocityPID(0x80, Kp, Ki, Kd, qpps);
  RC1.SetM1PositionPID(0x80, fsettings[3], fsettings[4], fsettings[5], static_cast<int>(fsettings[6]), fsettings[7], -10000, 10000); // change the position settings
  RC1.SetM2PositionPID(0x80, fsettings[3], fsettings[4], fsettings[5], static_cast<int>(fsettings[6]), fsettings[7], -10000, 10000);
  RC2.SetM1PositionPID(0x80, fsettings[3], fsettings[4], fsettings[5], static_cast<int>(fsettings[6]), fsettings[7], -10000, 10000);
  RC2.SetM2PositionPID(0x80, fsettings[3], fsettings[4], fsettings[5], static_cast<int>(fsettings[6]), fsettings[7], -10000, 10000);
}


// Writes val to the EEPROM memory at address
void MemWrite(int adr, float val) {
  EEPROM.put(adr, val);
}


// Retrieves len from serial port using pySerialTransfer, starting at position recievePOS
template<class T>
T * RetrieveSerial(size_t len, uint16_t & recievePOS) {
  size_t datasize = len; // sets size of data in the packet
  T * data = new T[datasize]; // creates an empty array of datasize
  for(size_t i = 0; i < datasize; i++) { // takes data from serial port and adds it to the packet
    recievePOS = rx.rxObj(data[i], recievePOS);
    //Serial.println(data[i]);
  }
  return data;
}

int * RetrieveTelemetry(RoboClaw &RC1, RoboClaw &RC2){
  //elapsedMillis telemMeasure = 0;
  // Retrieve Encoder count and speed - 8 vals
  // Retrieve Motor currents - 4 vals

  int * telemetryData = new int[14];

  // Retrieve Encoder counts
  uint8_t status1,status2,status3,status4;
  bool valid1=false,valid2=false,valid3=false,valid4=false;
  uint32_t count1=0, count2=0, count3=0, count4=0;
  for (int i = 0; i < 5; i ++) {
      count1 = RC1.ReadEncM1(0x80, &status1, &valid1);
      count2 = RC1.ReadEncM2(0x80, &status2, &valid2);
      count3 = RC2.ReadEncM1(0x80, &status3, &valid3);
      count4 = RC2.ReadEncM2(0x80, &status4, &valid4);
      if (valid1 && valid2 && valid3 && valid4) break;
      else Serial.println("invalid encoder readouts");
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
      speed1 = RC1.ReadSpeedM1(0x80, &status5, &valid5);
      speed2 = RC1.ReadSpeedM2(0x80, &status6, &valid6);
      speed3 = RC2.ReadSpeedM1(0x80, &status7, &valid7);
      speed4 = RC2.ReadSpeedM2(0x80, &status8, &valid8);
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
    rc1cval = RC1.ReadCurrents(0x80, c1, c2);
    rc2cval = RC2.ReadCurrents(0x80, c3, c4);
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
    v1 = RC1.ReadMainBatteryVoltage(0x80, &v1val);
    v2 = RC2.ReadMainBatteryVoltage(0x80, &v2val);
    if (v1val && v2val) break;
  }

  telemetryData[12] = (int)v1;
  telemetryData[13] = (int)v2;

  // for (size_t i = 0; i < 14; i++) {
  //   Serial.println(telemetryData[i]);
  // }
  //Serial.println(telemMeasure);
  return telemetryData;
}

void SendTelem(int * data, size_t len) {
  size_t sendSize = 0;
  for (size_t i = 0; i < len; i++) {
    sendSize = rx.txObj(data[i],sendSize);
  }
  rx.sendData(sendSize);
  delete[] data;
  return;
}

void WaitForRoboclaw(RoboClaw& RC) {
  uint8_t status;
  bool valid = false;
  int32_t dummy;
  for (int i = 0; i < 50; i++) {  // Try for ~2.5 seconds
    dummy = RC.ReadEncM1(0x80, &status, &valid);
    if (valid) {
      Serial.println("RoboClaw ready");
      return;
    }
    delay(50);
  } Serial.println("Roboclaw not ready");
  return;
}





