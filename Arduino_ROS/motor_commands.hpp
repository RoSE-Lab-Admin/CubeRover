#ifndef MOTOR
#define MOTOR




void motor_speed(int arg) {
  if (arg > 258) arg = 258;
  analogWrite(MOTOR_PIN, arg);
}

int get_motor_speed() {
  return analogRead(MOTOR_PIN);
}


#endif