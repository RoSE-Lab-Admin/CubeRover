#define BAUDRATE 57600



#include "commands.h"
#include "motor_commands.hpp"
#include <Servo.h>
#include "servo.h"



char chr;
char cmd;
short int arg = 0;
char argv1[32];
char argv2[32];
size_t index = 0;

int stepDelay = 20;
byte servoPins = STEER_PIN;
byte servoInitPosition = 90;

SweepServo steer;


void resetCommand() { // reset global variables
  cmd = ' ';
  arg = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  index = 0;
}

void runCommand() {
  long arg1 = atoi(argv1);
  long arg2 = atoi(argv2);

  switch (cmd) {
    case CAR:

      break;
    case SET_MOTOR_SPEED:
      motor_speed(arg1);
      Serial.println("OK");
      break;
    case GET_MOTOR_TELEM:
      Serial.println(get_motor_speed() + "\r");
      break;
    case RESET_ENCODERS:

      break;
    case SERVO_WRITE:
      steer.setTargetPosition(arg1);
      break;

    case SERVO_READ:
      Serial.println(steer.getServo().read() + "\r");
      break;

  }
}

void setup() {
  Serial.begin(BAUDRATE);

  resetCommand();

  steer.initServo(
  servoPins,
  stepDelay,
  servoInitPosition);

  Serial.print("setup complete");
}


void loop() {
  while (Serial.available()) {
    chr = Serial.read();

    if (chr == 13) { // standard ascii carrige return ('\r')
      if (arg == 1) argv1[index] = NULL; // set current arg index to null. terminates the rest of the arg
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();

    } else if (chr == ' ') { // if chr is space (' ') -> switch arg
      if (arg == 0) arg = 1; // move to arg 1
      else if (arg == 1) {
        argv1[index] = NULL;
        index = 0;
        arg = 2;
      }
      continue;
    }
    else {
      if (arg == 0) cmd = chr; // 
      else if (arg == 1) argv1[index++] = chr;
      else if (arg == 2) argv2[index++] = chr;
    }

  }
  steer.doSweep();
}
