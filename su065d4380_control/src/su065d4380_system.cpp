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

  const std::string port_name = this->info_.hardware_parameters["dev"];
  this->port_handler_ =
    std::make_shared<su065d4380_interface::PortHandler>(port_name);
  // Dry run for connection test
  if (!this->port_handler_->openPort()) {
    RCLCPP_FATAL(this->getLogger(), "Failed to open port");
    return CallbackReturn::ERROR;
  }
  // Send to stop
  this->port_handler_->closePort();

  const int ld_tmp =
    std::stoi(this->info_.hardware_parameters["left_rotation_direction"]);
  if (ld_tmp < 0) {
    this->left_rot_dir_ = -1.0;
  }
  const int rd_tmp =
    std::stoi(this->info_.hardware_parameters["right_rotation_direction"]);
  if (rd_tmp < 0) {
    this->right_rot_dir_ = -1.0;
  }

  this->reduction_ratio_ =
    std::stod(this->info_.hardware_parameters["reduction_ratio"]);

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

  if (this->info_.joints.at(
      static_cast<size_t>(WHEEL_IDX::LEFT)).name.find("left") ==
    std::string::npos)
  {
    RCLCPP_FATAL(
      this->getLogger(),
      "The index for left wheel expected to be %d",
      static_cast<int>(WHEEL_IDX::LEFT));
    return CallbackReturn::ERROR;
  }

  if (this->info_.joints.at(
      static_cast<size_t>(WHEEL_IDX::RIGHT)).name.find("right") ==
    std::string::npos)
  {
    RCLCPP_FATAL(
      this->getLogger(),
      "The index  for wright wheel expected to be %d",
      static_cast<int>(WHEEL_IDX::RIGHT));
    return CallbackReturn::ERROR;
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
  this->port_handler_->openPort();
  this->packet_handler_ =
    std::make_unique<su065d4380_interface::PacketHandler>(
    this->port_handler_);

  // Set default value
  for (size_t i = 0; i < this->hw_positions_.size(); ++i) {
    if (std::isnan(this->hw_positions_.at(i))) {
      this->hw_positions_.at(i) = 0.0;
      this->hw_velocities_.at(i) = 0.0;
      this->hw_commands_.at(i) = 0.0;
    }
  }

  // STOP motor
  this->packet_handler_->sendVelocityCommand(0.0, 0.0);

  RCLCPP_INFO(this->getLogger(), "Successfully activated!");
  return CallbackReturn::SUCCESS;
}

CallbackReturn SU065D4380System::on_deactivate(const rclcpp_lifecycle::State &)
{
  // STOP motor
  this->packet_handler_->sendVelocityCommand(0.0, 0.0);
  this->port_handler_->closePort();
  this->packet_handler_ = nullptr;
  RCLCPP_INFO(this->getLogger(), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type SU065D4380System::read()
{
  while (this->port_handler_->getBytesAvailable() > 0) {
    this->packet_handler_->recvCommand();
  }

  // TODO(anyone): Fix it for continuous position change
  this->hw_positions_.at(static_cast<size_t>(WHEEL_IDX::LEFT)) =
    this->packet_handler_->getLeftPosition();
  this->hw_positions_.at(static_cast<size_t>(WHEEL_IDX::RIGHT)) =
    this->packet_handler_->getRightPosition();

  this->hw_velocities_.at(static_cast<size_t>(WHEEL_IDX::LEFT)) =
    this->packet_handler_->getLeftVelocity();
  this->hw_velocities_.at(static_cast<size_t>(WHEEL_IDX::RIGHT)) =
    this->packet_handler_->getRightVelocity();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SU065D4380System::write()
{
  const double left_rps =
    this->reduction_ratio_ *
    this->left_rot_dir_ *
    this->hw_commands_.at(static_cast<size_t>(WHEEL_IDX::LEFT));
  const double right_rps =
    this->reduction_ratio_ *
    this->right_rot_dir_ *
    this->hw_commands_.at(static_cast<size_t>(WHEEL_IDX::RIGHT));

  if (!this->packet_handler_->sendVelocityCommand(right_rps, left_rps)) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Failed to send velocity command");
    return hardware_interface::return_type::ERROR;
  }
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
