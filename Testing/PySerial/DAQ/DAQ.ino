#include "motor_driver.h"
#include <elapsedMillis.h>
#include "RoboClaw.h"

#define BAUDRATE 115200
#define RC1_SERIAL Serial1
#define RC2_SERIAL Serial3

RoboClaw* ROBOCLAW_1 = new RoboClaw(&RC1_SERIAL, 10000);
RoboClaw* ROBOCLAW_2 = new RoboClaw(&RC2_SERIAL, 10000);

void setup() {
  // start serial on usb port
  Serial.begin(BAUDRATE);

  // setup motor controllers
  ROBOCLAW_1->begin(38400);
  ROBOCLAW_2->begin(38400);
  init_motor_controllers(ROBOCLAW_1, ROBOCLAW_2);

  // turn on LED status light
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
 
  //Serial.print("setup complete");

  encoder_reset();
  //Serial.print("Encoders reset");

  digitalWrite(13,HIGH);
}

void loop() {
    String telem = get_telemetry();
    Serial.print(telem);
    Serial.flush();
}