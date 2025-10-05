#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <cmath>


class wheel {
    public:
        wheel(float enc_count_per_rev);
        void updatePos(int encPos);
        void updateVel(int encVel);
        void updateCur(int mCur);
        void updateVolt(int mVolt);
        int cmd_to_enc();
        double pos_ = 0; // rad
        double vel_ = 0; // rad/s
        double current_ = 0; // amp
        double voltage_ = 0; // voltage
        double cmd_ = 0;
    private:
        float rads_per_ct_ = 0;
};


wheel::wheel(float enc_count_per_rev) {
    rads_per_ct_ = (2 * M_PI) / enc_count_per_rev;
}

void wheel::updatePos(int encPos){
    pos_ = encPos * rads_per_ct_;
}

void wheel::updateVel(int encVel){
    vel_ = encVel * rads_per_ct_;
}

void wheel::updateCur(int mCur){
    // RH: recorded in increments of 10mA (100th of amp)
    current_ = mCur / 100.0;
}

void wheel::updateVolt(int mVolt){
    // RH: recorded in increments of 100mV (10th of volt)
    voltage_ = mVolt / 10.0;
}

int wheel::cmd_to_enc() {
    return static_cast<int>(cmd_ / rads_per_ct_);
}




#endif
