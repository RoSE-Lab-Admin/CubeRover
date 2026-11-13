#include "commands.h"
#include "motor_driver.h"

#include <elapsedMillis.h>
#include <RoboClaw.h>
#include <EEPROM.h>

#define BAUDRATE 115200
#define RC1_SERIAL Serial1
#define RC2_SERIAL Serial3

#define WHEEL_DIAMETER_M 0.303
#define COUNTS_PER_REV 2000.0
#define DIR_SIGN 1
#define TRAVEL_M 4.0
#define OVERTRAVEL_M 0.25
#define CRUISE_MPS 0.50
#define CONTROL_PERIOD_MS 20
#define FEEDBACK_PERIOD_MS 100
#define DEFAULT_PID_P 100
#define DEFAULT_PID_I 50
#define DEFAULT_PID_D 25

const float COUNTS_PER_M = COUNTS_PER_REV / (PI * WHEEL_DIAMETER_M);
const long CRUISE_COUNTS = (long)(CRUISE_MPS * COUNTS_PER_M);

char chr = ' ';
char cmd = ' ';
byte arg = 0;
char argv1[32];
char argv2[32];
char argv3[32];
size_t i = 0;

enum State { IDLE, MOVING };
State state = IDLE;
int8_t heading = 1;
int32_t enc_counts = 0;
int32_t speed_counts = 0;

RoboClaw* ROBOCLAW_1 = new RoboClaw(&RC1_SERIAL, 10000);
RoboClaw* ROBOCLAW_2 = new RoboClaw(&RC2_SERIAL, 10000);

elapsedMillis control_timer = 0;
elapsedMillis feedback_timer = 0;


void reset_command() {
  cmd = ' ';
  arg = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  memset(argv3, 0, sizeof(argv3));
  i = 0;
}


void run_command() {
  switch (cmd) {
    case START_TEST: {
      if (state == IDLE) {
        encoder_reset();
        heading = 1;
        control_timer = 0;
        set_motor_speed(1, heading * CRUISE_COUNTS * DIR_SIGN);
        state = MOVING;
      }
      break;
    }
    case STOP_TEST: {
      set_motor_speed(1, 0);
      state = IDLE;
      break;
    }
  }
}


bool parse_telemetry() {
  String telem = get_telemetry();
  telem.trim();
  if (telem.length() < 1 || telem[0] != 'e') return false;
  long enc = 0, speed = 0;
  if (sscanf(telem.c_str(), "e %ld %*ld %*ld %*ld %ld", &enc, &speed) != 2) return false;
  enc_counts = (int32_t)enc;
  speed_counts = (int32_t)speed;
  return true;
}


void test_fault(const char* msg) {
  set_motor_speeds(0,0);
  state = IDLE;
  while (true) {
    digitalWrite(13,HIGH);
    delay(500);
    digitalWrite(13,LOW);
    delay(500);
    Serial.println(msg);
  }
}


void control_tick() {
  if (state == IDLE) return;

  if (feedback_timer >= FEEDBACK_PERIOD_MS) {
    feedback_timer = 0;
    parse_telemetry();
  }

  float pos = (float)enc_counts / COUNTS_PER_M * DIR_SIGN;
  if (pos > TRAVEL_M + OVERTRAVEL_M || pos < -OVERTRAVEL_M) test_fault("TRAVEL LIMIT EXCEEDED!");

  if (state == MOVING) {
    if ((heading > 0 && pos >= TRAVEL_M) || (heading < 0 && pos <= 0)) {
      heading = -heading;
      set_motor_speed(1, heading * CRUISE_COUNTS * DIR_SIGN);
    }
  }
}


void setup() {
  Serial.begin(BAUDRATE);

  ROBOCLAW_1->begin(38400);
  ROBOCLAW_2->begin(38400);
  init_motor_controllers(ROBOCLAW_1, ROBOCLAW_2);

  float pcheck;
  EEPROM.get(0, pcheck);
  if (isnan(pcheck) || pcheck == 0) {
    pid_set(DEFAULT_PID_P, DEFAULT_PID_I, DEFAULT_PID_D);
    init_motor_controllers(ROBOCLAW_1, ROBOCLAW_2);
  }

  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);

  encoder_reset();
}


void loop() {
  while (Serial.available()) {
    chr = Serial.read();

    if (chr == 13) {
      if (arg == 1) argv1[i] = '\0';
      else if (arg == 2) argv2[i] = '\0';
      else if (arg == 3) argv3[i] = '\0';
      run_command();
      reset_command();
    }
    else if (chr == ' ') {
      if (arg == 0) arg = 1;
      else if (arg == 1) {
        argv1[i] = '\0';
        i = 0;
        arg = 2;
      }
      else if (arg == 2) {
        argv2[i] = '\0';
        i = 0;
        arg = 3;
      }
      continue;
    }
    else {
      if (arg == 0) cmd = chr;
      else if (arg == 1) argv1[i++] = chr;
      else if (arg == 2) argv2[i++] = chr;
      else if (arg == 3) argv3[i++] = chr;
    }
  }

  if (control_timer >= CONTROL_PERIOD_MS) {
    control_timer = 0;
    control_tick();
  }
}
