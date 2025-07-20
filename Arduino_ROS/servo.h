#ifndef SERVO_H
#define SERVO_H


class SweepServo
{
  public:
    SweepServo();
    void initServo(
        int servoPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    Servo getServo();

  private:
    Servo servo;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};


#endif