#ifndef WHEEL_HPP
#define WHEEL_HPP

#include <string>
#include <numbers>
#include <cmath>


class wheel {
    public:
        wheel(float enc_count_per_rev);
        void updatePos(int encPos);
        void updateVel(int encVel);
        void updateCur(int mCur);
        void updateVolt(int mVolt);
        int cmd_to_end();
        float pos_ = 0; // rad
        float vel_ = 0; // rad/s
        float current_ = 0; // amp
        float voltage_ = 0; // voltage
        float cmd_ = 0;
    private:
        float rads_per_ct_ = 0;
}


wheel::wheel(float enc_count_per_rev) {
    rads_per_ct_ = (2*std::numbers::pi) / enc_count_per_rev;
}

void wheel::updatePos(int encPos){
    pos_ = std::fmod(encPos * rads_per_ct_, 2*std::numbers::pi);
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

void wh




#endif