#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"

TEST(package_name, a_first_test)
{
  // Some test that uses ROS
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv); // <--- Initialize ROS 2
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();       // <--- Clean up ROS 2
  return result;
}