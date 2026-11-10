#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <cmath>


class wheel {
    public:
        wheel(double enc_count_per_rev, double torque_constant);
        void updatePos(int encPos);
        void updateVel(int encVel);
        void updateCur(int mCur);
        void updateVolt(int mVolt);
        void updatePWM(int mPWM);
        int cmd_to_enc();
        double pos_ = 0; // rad
        double vel_ = 0; // rad/s
        double current_ = 0; // amp
        double effort_ = 0; // N*m, current_ * torque_constant_
        double voltage_ = 0; // voltage
        double pwm_ = 0; // motor pwm
        double cmd_ = 0;
    private:
        double rads_per_ct_ = 0;
        double torque_constant_ = 0; // N*m per A
};


wheel::wheel(double enc_count_per_rev, double torque_constant) {
    rads_per_ct_ = (2 * M_PI) / enc_count_per_rev;
    torque_constant_ = torque_constant;
}

void wheel::updatePos(int encPos){
    pos_ = encPos * rads_per_ct_;
}

void wheel::updateVel(int encVel){
    vel_ = encVel * rads_per_ct_;
}

void wheel::updateCur(int mCur){
    // RoboClaw Manual: 49 - Read Motor Currents
    // RH: recorded in increments of 10mA (100th of amp)
    current_ = mCur / 100.0;
    // Effort from the motor characterization (Motor Characterization.xlsx):
    // no offset or sign handling, so it reads ~0.45 N*m at no-load current.
    effort_ = current_ * torque_constant_;
}

void wheel::updateVolt(int mVolt){
    // RoboClaw Manual: 24 - Read Main Battery Voltage Level
    // RH: recorded in increments of 100mV (10th of volt)
    voltage_ = mVolt / 10.0;
}

void wheel::updatePWM(int mPWM) {
    // RoboClaw Manual: 48 - Read Motor PWM values
    // Duty cycle percent is calculated by dividing the Value by 327.67.
    pwm_ = mPWM / 327.67;
}

int wheel::cmd_to_enc() {
    return static_cast<int>(cmd_ / rads_per_ct_);
}




#endif
