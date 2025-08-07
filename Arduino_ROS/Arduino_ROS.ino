// Heavy inspiration from the ros_arduino_bridge repo: https://github.com/hbrobotics/ros_arduino_bridge

// define header files to handle commands
#include "commands.h"
#include "motor_driver.h"

//include Arduino packages
#include <elapsedMillis.h>
#include <RoboClaw.h>

// global definitions
#define BAUDRATE 57600
#define RC1_SERIAL Serial1
#define RC2_SERIAL Serial3
#define MOTOR_TIMEOUT 2000

// global variables
char chr = ' ';           // stores current character from serial
char cmd = ' ';           // stores command type
byte arg = 0;             // stores argument number
char argv1[32];           // char array to temporarily store argument 1
char argv2[32];           // ~~~ argument 2
char argv3[32];
size_t i = 0;             // index in argv variable

// Roboclaw definitions
RoboClaw* ROBOCLAW_1 = new RoboClaw(&RC1_SERIAL, 10000);
RoboClaw* ROBOCLAW_2 = new RoboClaw(&RC2_SERIAL, 10000);

// timer to handle motor timeout if 2 seconds has passed without motor command
elapsedMillis motor_timeout = 0;


void reset_command() { // reset global variables
  cmd = ' ';
  arg = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv2));
  i = 0;
}

// run current command
void run_command() {
  long arg1 = atoi(argv1);
  long arg2 = atoi(argv2);
  long arg3 = atoi(argv3);

  switch (cmd) {
    case SET_MOTOR_SPEEDS: {
      set_motor_speeds(arg1, arg2, arg3);
      motor_timeout = 0;
      Serial.println("ok");
      break;
    }
    case SET_MOTOR_SPEED: {
      set_motor_speed(arg1, arg2, arg3);
      motor_timeout = 0;
      Serial.println("ok");
      break;
    }
    case GET_TELEM: {
      String telem = get_telemetry();
      Serial.println(telem);
      break;
    }
    case RESET_ENCODERS: {
      encoder_reset();
      Serial.println("encoders reset");
      break;
    }
    case PID: {
      pid_set(arg1, arg2, arg3);
      init_motor_controllers(ROBOCLAW_1, ROBOCLAW_2);
      Serial.println("PID values set and motor controllers reinitialized");
      break;
    }
  }
}

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

  Serial.print("setup complete");
}


void loop() {
  while (Serial.available()) {
    chr = Serial.read();

    if (chr == 13) { // standard ascii carrige return ('\r')
      if (arg == 1) argv1[i] = '\0'; // set current arg index to null. terminates the rest of the arg
      else if (arg == 2) argv2[i] = '\0';
      else if (arg == 3) argv3[i] = '\0';
      run_command();
      reset_command();
    } 
    else if (chr == ' ') { // if chr is space (' ') -> switch arg
      if (arg == 0) arg = 1; // move to arg 1
      else if (arg == 1) { // move to arg 2, stop current arg1
        argv1[i] = '\0';
        i = 0;
        arg = 2;
      }
      else if (arg == 2) { // move to arg 3, stop current arg2
        argv2[i] = '\0';
        i = 0;
        arg = 3;
      }
      continue;
    }
    else {
      if (arg == 0) cmd = chr; // write first char to command type
      else if (arg == 1) argv1[i++] = chr;  // write to first arg list
      else if (arg == 2) argv2[i++] = chr;  // ~~~ second arg
      else if (arg == 3) argv3[i++] = chr;  // ~~~ third arg
    }
  }
  if (motor_timeout > MOTOR_TIMEOUT) { //sets motor speeds to 0 if more than 2 seconds has ellapsed
    set_motor_speeds(0,0, 500);
  }

}
