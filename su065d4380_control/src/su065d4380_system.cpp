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

CallbackReturn SU065D4380System::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const std::string port_name = this->info_.hardware_parameters["dev"];
  this->interface_ = std::make_unique<Interface>(port_name);
  this->interface_->on_init();

  this->state_positions_.resize(
    this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  this->state_velocities_.resize(
    this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  this->command_velocities_.resize(
    this->info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : this->info_.joints) {
    if (joint.command_interfaces.at(0).name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        this->getLogger(),
        "Joint '%s' have '%s' command interface found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces.at(0).name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.at(0).name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
        this->getLogger(), "Joint '%s' have '%s' state interface. '%s' expected.'",
        joint.name.c_str(), joint.state_interfaces.at(0).name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.at(1).name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(
        this->getLogger(),
        "Joint '%s' have '%s' state interface. '%s' expected.'",
        joint.name.c_str(), joint.state_interfaces.at(1).name.c_str(),
        hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  for (const auto & transmission : this->info_.transmissions) {
    for (const auto & joint : transmission.joints) {
      if (joint.name.find("left") != std::string::npos) {
        this->left_reduction_ratio_ = joint.mechanical_reduction;
      }
      if (joint.name.find("right") != std::string::npos) {
        this->right_reduction_ratio_ = joint.mechanical_reduction;
      }
    }
  }

  RCLCPP_INFO(
    this->getLogger(), "Reduction Ratio: %.3lf, %.3lf",
    this->left_reduction_ratio_, this->right_reduction_ratio_);

  if (this->info_.joints.at(0).name.find("left") == std::string::npos) {
    RCLCPP_FATAL(this->getLogger(), "The index for left wheel expected to be 0");
    return CallbackReturn::ERROR;
  }

  if (this->info_.joints.at(1).name.find("right") == std::string::npos) {
    RCLCPP_FATAL(this->getLogger(), "The index for wright wheel expected to be 1");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SU065D4380System::on_configure(const State & state)
{
  if (this->interface_->on_configure(state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SU065D4380System::on_activate(const State & state)
{
  if (this->interface_->on_activate(state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // Set default value
  for (size_t i = 0; i < this->state_positions_.size(); ++i) {
    if (std::isnan(this->state_positions_.at(i))) {
      this->state_positions_.at(i) = 0.0;
      this->state_velocities_.at(i) = 0.0;
      this->command_velocities_.at(i) = 0.0;
    }
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SU065D4380System::on_deactivate(const State & state)
{
  if (this->interface_->on_deactivate(state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SU065D4380System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interface;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interface.emplace_back(
      hardware_interface::StateInterface(
        this->info_.joints.at(i).name, hardware_interface::HW_IF_POSITION,
        &this->state_positions_.at(i)));
    state_interface.emplace_back(
      hardware_interface::StateInterface(
        this->info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY,
        &this->state_velocities_.at(i)));
  }
  return state_interface;
}

std::vector<hardware_interface::CommandInterface> SU065D4380System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < this->info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        this->info_.joints.at(i).name, hardware_interface::HW_IF_VELOCITY,
        &this->command_velocities_.at(i)));
  }

  return command_interfaces;
}

return_type SU065D4380System::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  this->interface_->read();

  this->state_positions_.at(0) = this->interface_->getLeftRadian();
  this->state_positions_.at(1) = this->interface_->getRightRadian();

  this->state_velocities_.at(0) = this->interface_->getLeftVelocity();
  this->state_velocities_.at(1) = this->interface_->getRightVelocity();

  return return_type::OK;
}

return_type SU065D4380System::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  this->interface_->setVelocity(this->command_velocities_.at(0), this->command_velocities_.at(1));
  this->interface_->write();

  return return_type::OK;
}

const rclcpp::Logger SU065D4380System::getLogger() noexcept
{
  return rclcpp::get_logger("SU065D4380System");
}
}  // namespace su065d4380_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(su065d4380_control::SU065D4380System, hardware_interface::SystemInterface)
