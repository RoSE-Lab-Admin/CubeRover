// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "roseybot_arduino/roseybot_system.hpp"
#include "roseybot_arduino/ROS_Arduino.hpp"
#include "roseybot_arduino/wheel.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"



namespace roseybot_arduino_interface
{

hardware_interface::CallbackReturn RoseyBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // BEGIN: get hardware parameters
  BAUD_ = std::stoi(info_.hardware_parameters["baud"]);
  TIMEOUT_MS_ = std::stoi(info_.hardware_parameters["timeout_ms"]);
  DEVICE_ = info_.hardware_parameters["device"];
  ENC_PER_REV_ = std::stof(info_.hardware_parameters["enc/rev"]);
  STD_ACCEL = std::stof(info_.hardware_parameters["std_accel"]);
  // END

  // create class for handling arduino comm_
  comm_ = new ArduinoComms(get_logger());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    
    // make sure each wheel only has 1 command interface
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 4)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 4 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // create wheel obj and push back 
    wheel_map_[joint.name] = std::make_unique<wheel>(ENC_PER_REV_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}




std::vector<hardware_interface::StateInterface> RoseyBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
    auto emplace_state = [&](std::string jointName) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        jointName, hardware_interface::HW_IF_POSITION, &wheel_map_[jointName]->pos_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        jointName, hardware_interface::HW_IF_VELOCITY, &wheel_map_[jointName]->vel_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        jointName, "current", &wheel_map_[jointName]->current_));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        jointName, "voltage", &wheel_map_[jointName]->voltage_));
    };
    
    for (const auto & joint : info_.joints) {
      emplace_state(joint.name);
    }

  return state_interfaces;
}



std::vector<hardware_interface::CommandInterface> RoseyBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
    auto emplace_command = [&](std::string jointName){
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        jointName, hardware_interface::HW_IF_VELOCITY, &wheel_map_[jointName]->cmd_));
    };
  
    for (const auto & joint : info_.joints) {
      emplace_command(joint.name);
    }
  return command_interfaces;
}



hardware_interface::CallbackReturn RoseyBotSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring ...please wait...");
  // connect to arduino
  if (comm_->connected()){
    comm_->disconnect();
  }
  try{
    comm_->connect(DEVICE_, BAUD_, TIMEOUT_MS_);
  } catch (const std::runtime_error& e) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::CallbackReturn RoseyBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  // check if serial is activated
  if (!comm_->connected()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // command and state should be equal when starting -> prevent discontinuities
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, get_state(name));
  }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::CallbackReturn RoseyBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  
  RCLCPP_INFO(get_logger(), "Deactivating ...please wait...");

  //disconnect arduino
  comm_->disconnect();

  RCLCPP_INFO(get_logger(), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::return_type RoseyBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  std::vector<int> telemVals(12);
  comm_->read_telem_values(telemVals);

  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const auto & joint = info_.joints[i];
    wheel_map_[joint.name]->updatePos(telemVals[i]);
    wheel_map_[joint.name]->updateVel(telemVals[i + 4]);
    wheel_map_[joint.name]->updateCur(telemVals[i + 8]);
  }

  return hardware_interface::return_type::OK;
}




hardware_interface::return_type roseybot_arduino_interface ::RoseyBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  
  comm_->set_motor_values(wheel_map_[info_.joints[0].name]->cmd_to_enc(), wheel_map_[info_.joints[2].name]->cmd_to_enc());

  return hardware_interface::return_type::OK;
}



}  // namespace roseybot_arduino_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  roseybot_arduino_interface::RoseyBotSystemHardware, hardware_interface::SystemInterface)
