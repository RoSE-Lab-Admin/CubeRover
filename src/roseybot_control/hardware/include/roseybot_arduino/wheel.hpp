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
    pos_ = std::fmod(encPos * rads_per_ct_, 2*M_PI);
}

void wheel::updateVel(int encVel){
    vel_ = encVel * rads_per_ct_;
}

void wheel::updateCur(int mCur){
    current_ = mCur / 10.0;
}

void wheel::updateVolt(int mVolt){
    voltage_ = mVolt / 10.0;
}

int wheel::cmd_to_enc() {
    return static_cast<int>(cmd_ / rads_per_ct_);
}




#endif