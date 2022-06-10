// Copyright 2022 HarvestX Inc.
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

#include "su065d4380_control/su065d4380_system.hpp"

namespace su065d4380_control
{

CallbackReturn SU065D4380System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  this->hw_positions_.resize(
    this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  this->hw_velocities_.resize(
    this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  this->hw_commands_.resize(
    this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : this->info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(
        this->getLogger(),
        "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces.at(0).name !=
      hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        this->getLogger(),
        "Joint '%s' have '%s' command interface found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces.at(0).name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(
        this->getLogger(),
        "Joint '%s' has %zu state interface. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.at(0).name !=
      hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        this->getLogger(),
        "Joint '%s' have '%s' state interface. '%s' expected.'",
        joint.name.c_str(), joint.state_interfaces.at(0).name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.at(1).name !=
      hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        this->getLogger(),
        "Joint '%s' have '%s' state interface. '%s' expected.'",
        joint.name.c_str(), joint.state_interfaces.at(1).name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SU065D4380System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interface;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interface.emplace_back(
      hardware_interface::StateInterface(
        this->info_.joints.at(i).name,
        hardware_interface::HW_IF_POSITION,
        &hw_positions_.at(i)));
    state_interface.emplace_back(
      hardware_interface::StateInterface(
        this->info_.joints.at(i).name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocities_.at(i)));
  }
  return state_interface;
}

std::vector<hardware_interface::CommandInterface>
SU065D4380System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < this->info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->info_.joints.at(i).name,
        hardware_interface::HW_IF_VELOCITY,
        &this->hw_commands_.at(i)));
  }

  return command_interfaces;
}

CallbackReturn SU065D4380System::on_activate(const rclcpp_lifecycle::State &)
{
  // Set default value
  for (size_t i = 0; i < this->hw_positions_.size(); ++i) {
    if (std::isnan(this->hw_positions_.at(i))) {
      this->hw_positions_.at(i) = 0.0;
      this->hw_velocities_.at(i) = 0.0;
      this->hw_commands_.at(i) = 0.0;
    }
  }

  RCLCPP_INFO(this->getLogger(), "Successfully activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SU065D4380System::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->getLogger(), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type SU065D4380System::read()
{
  for (size_t i = 0; i < hw_commands_.size(); ++i) {

  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SU065D4380System::write()
{
  return hardware_interface::return_type::OK;
}

rclcpp::Logger SU065D4380System::getLogger()
{
  return rclcpp::get_logger("XV3Control");
}
}  // namespace su065d4380_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  su065d4380_control::SU065D4380System,
  hardware_interface::SystemInterface)
