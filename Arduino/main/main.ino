
// Including packages
#include <RingBuf.h>
#include <elapsedMillis.h>
#include <SerialTransfer.h>
#include <RoboClaw.h>
#include "ControlPacket.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h> // soon to be removed and ported to the PI



// Robot Parameters
// float Kp = 11.37910;    // proportional constant for velocity PID
// float Ki = 0.345;   // integral constant for velocity PID
// float Kd = 0;   // derivative constant for velocity PID
float qpps = 3400; // countable quadrature pulses per second -> found using roboclaw's basicMicro tool


Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);


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
  ROBOCLAW_1.begin(38400);
  ROBOCLAW_2.begin(38400);

  // turn status LED on
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
  
  // Init roboclaw values from memory
  MemSetup(ROBOCLAW_1, ROBOCLAW_2);

  delay(100);

  // if (!bno.begin())
  // {
  //   for(int i = 0; i < 20; i++) {
  //     if (!bno.begin()) {
  //       Serial.print("No BNO055 detected");
  //       digitalWrite(13,!digitalRead(13));
  //       delay(500);
  //     }
  //   }
  // }
  //bno.setExtCrystalUse(true);
  delay(3000);
  // Serial8.begin(38400); // GPIO pins for Raspberry Pi
  rx.begin(Serial);
  Serial.println("Complete!");
}


ControlPacket * control = nullptr; // control pointer variable to keep track of what command is being done
RingBuf<ControlPacket*, 20> packetBuff; // packet buffer to keep commands if one is currently being resolved
elapsedMillis dt = 0;
elapsedMillis sendTimer = 0;
float hPos = 0;
float hVel = 0;
bool dataRequest = false;

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

  if (sendTimer > 50) {
    SendTelem(RetrieveTelemetry(ROBOCLAW_1, ROBOCLAW_2, bno), 12);
    sendTimer = 0;
    //Serial.print("Running");
  }
  // delay(20);
}


ControlPacket* SerialDecode () {
  uint16_t recievePOS = 0; // stores position of iterator in recieving buffer
  char ID; // stores ID of current packet decoder
  ControlPacket * controlTemp = nullptr; // temp pointer to decoded packet
  recievePOS = rx.rxObj(ID, recievePOS); // store ID char
  //Serial.print(ID);
  if (ID == 'V') { // Velocity control
    controlTemp = new VelPID(RetrieveSerial<int>(7, recievePOS)); // creates new packet of type Velocity
  } else if (ID == 'P') { // Distance / Position control
    //Serial.write("Distance!");
    controlTemp = new PosPID(RetrieveSerial<int>(2,recievePOS));
  } 
  // else if (ID == 'T') {
  //   //Serial.write("Turning!");
  //   controlTemp = new AngPID(RetrieveSerial<float>(3,recievePOS), bno, acceleration, deacceleration);
  // }
   else if (ID == 'E') { // stops the rover and empties the packet buffer -> ideally for emergency / resetting the rover --> FIX
    if (control != nullptr) {
      //Serial.println("STOP");
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
    //Serial.println(fsettings[i/4]);
  }
  RC1.SetM1VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps); // change the velocity settings
  RC1.SetM2VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps);
  RC2.SetM1VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps);
  RC2.SetM1VelocityPID(0x80, fsettings[0], fsettings[1], fsettings[2], qpps);
  RC1.SetM1PositionPID(0x80, fsettings[3], fsettings[4], fsettings[5], static_cast<int>(fsettings[6]), fsettings[7], -10000, 10000); // change the position settings
  RC1.SetM2PositionPID(0x80, fsettings[3], fsettings[4], fsettings[5], static_cast<int>(fsettings[6]), fsettings[7], -10000, 10000);
  RC2.SetM1PositionPID(0x80, fsettings[3], fsettings[4], fsettings[5], static_cast<int>(fsettings[6]), fsettings[7], -10000, 10000);
  RC2.SetM2PositionPID(0x80, fsettings[3], fsettings[4], fsettings[5], static_cast<int>(fsettings[6]), fsettings[7], -10000, 10000);
}


// Writes val to the EEPROM memory at address
void MemWrite(int adr, float val) {
  EEPROM.put(adr, val);
  // Serial.println(adr);
  // Serial.println(val);
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

int * RetrieveTelemetry(RoboClaw &RC1, RoboClaw &RC2, Adafruit_BNO055 &IMU){
  // Retrieve Encoder count and speed - 8 vals
  // Retrieve Motor currents - 4 vals

  int * telemetryData = new int[12];

  // Retrieve Encoder counts
  telemetryData[0] = RC1.ReadEncM1(0x80);
  telemetryData[1] = RC1.ReadEncM2(0x80);
  telemetryData[2] = RC2.ReadEncM1(0x80);
  telemetryData[3] = RC2.ReadEncM2(0x80);

  // Retrieve Encoder velocities
  telemetryData[4] = RC1.ReadSpeedM1(0x80);
  telemetryData[5] = RC1.ReadSpeedM2(0x80);
  telemetryData[6] = RC2.ReadSpeedM1(0x80);
  telemetryData[7] = RC2.ReadSpeedM2(0x80);

  int16_t c1,c2,c3,c4; // fix? currnetly currents are not correct values
  RC1.ReadCurrents(0x80, c1, c2);
  RC2.ReadCurrents(0x80, c3, c4);
  telemetryData[8] = static_cast<float>(c1);
  telemetryData[9] = static_cast<float>(c2);
  telemetryData[10] = static_cast<float>(c3);
  telemetryData[11] = static_cast<float>(c4);

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





