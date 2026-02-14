#include <gtest/gtest.h>
#include <cmath>
#include "roseybot_arduino/wheel.hpp"


// --------------------------------------------------------
// Standalone Tests (No Fixture needed)
// --------------------------------------------------------
TEST(WheelConstructorTest, ZeroCountsThrowException) {
    // Expect the constructor to throw std::invalid_argument
    // when passed 0.0
    EXPECT_THROW({
        wheel w(0.0);
    }, std::invalid_argument);
}

TEST(WheelConstructorTest, NegativeCountsThrowException) {
    // Expect the constructor to throw std::invalid_argument
    // when passed negative number
    EXPECT_THROW({
        wheel w(-100.0);
    }, std::invalid_argument);
}


// --------------------------------------------------------
// Fixture 1: Simple Math (Ratio = 1.0)
// Good for checking logic without needing a calculator
// --------------------------------------------------------
class SimpleWheelTest : public testing::Test {
protected:
  // Ratio 1.0 means: 1 encoder count = 1 radian
  wheel w_simple_{2 * (float)M_PI}; 
};

TEST_F(SimpleWheelTest, PosAndVelMath) {
  // Since ratio is 1.0, counts should equal radians exactly
  w_simple_.updatePos(10);
  EXPECT_DOUBLE_EQ(w_simple_.pos_, 10.0);

  w_simple_.updateVel(5);
  EXPECT_DOUBLE_EQ(w_simple_.vel_, 5.0);
}

TEST_F(SimpleWheelTest, CmdToEncTruncation) {
  // 0.99 radians / 1.0 ratio = 0.99 -> round long -> 1
  w_simple_.cmd_ = 0.99;
  EXPECT_EQ(w_simple_.cmd_to_enc(), 1);

  // 1.0 radians / 1.0 ratio = 1.0 -> cast to int -> 1
  w_simple_.cmd_ = 1.0;
  EXPECT_EQ(w_simple_.cmd_to_enc(), 1);
}

// Conversion Tests (Current/Voltage)
// --------------------------------------------------------
TEST_F(SimpleWheelTest, CurrentIn10mA) {
  // Test input in 10 mA increments
  // Expect 150 * 10 mA == 1.5 A
  int inputMilliAmps = 150;
  w_simple_.updateCur(inputMilliAmps);
  EXPECT_DOUBLE_EQ(w_simple_.current_, 1.5);
}

TEST_F(SimpleWheelTest, VoltageIn100mV) {
  // Test input in 100 mV increments
  // Expect: 15 * 100 mV == 1.5 V
  int inputMilliVolts = 15; 
  w_simple_.updateVolt(inputMilliVolts);
  EXPECT_DOUBLE_EQ(w_simple_.voltage_, 1.5);
}


// --------------------------------------------------------
// Fixture 2: Realistic Math (Ratio != 1.0)
// Ensures we didn't mix up * and / in the code
// --------------------------------------------------------
class RealisticWheelTest : public testing::Test {
protected:
  // 1000 counts per rev -> 1 count = 0.00628 radians
  wheel w_real_{1000.0}; 
};

TEST_F(RealisticWheelTest, CheckConversionFactors) {
  // If we input 1000 counts (one full rotation), we expect 2*PI radians
  w_real_.updatePos(1000);
  EXPECT_NEAR(w_real_.pos_, 2 * M_PI, 0.0001);

  // If we command 2*PI radians (one full rotation), we expect 1000 counts
  w_real_.cmd_ = 2 * M_PI;
  EXPECT_EQ(w_real_.cmd_to_enc(), 1000);
}
