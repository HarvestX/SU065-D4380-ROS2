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
    std::make_unique<su065d4380_interface::PortHandler>(port_name);
  // Dry run for connection test
  if (!this->port_handler_->openPort()) {
    RCLCPP_FATAL(this->getLogger(), "Failed to open port");
    return CallbackReturn::ERROR;
  }
  this->port_handler_->closePort();

  double left_rot_dir = 1.0, right_rot_dir = 1.0;
  const int ld_tmp =
    std::stoi(this->info_.hardware_parameters["left_rotation_direction"]);
  if (ld_tmp < 0) {
    left_rot_dir = -1.0;
  }
  const int rd_tmp =
    std::stoi(this->info_.hardware_parameters["right_rotation_direction"]);
  if (rd_tmp < 0) {
    right_rot_dir = -1.0;
  }

  const double reduction_ratio =
    std::stod(this->info_.hardware_parameters["reduction_ratio"]);
  this->left_coefficient_ = reduction_ratio * left_rot_dir;
  this->right_coefficient_ = reduction_ratio * right_rot_dir;

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

  if (this->info_.joints.at(LEFT_WHEEL_IDX).name.find("left") ==
    std::string::npos)
  {
    RCLCPP_FATAL(
      this->getLogger(),
      "The index for left wheel expected to be %zu",
      LEFT_WHEEL_IDX);
    return CallbackReturn::ERROR;
  }

  if (this->info_.joints.at(RIGHT_WHEEL_IDX).name.find("right") ==
    std::string::npos)
  {
    RCLCPP_FATAL(
      this->getLogger(),
      "The index for wright wheel expected to be %zu",
      RIGHT_WHEEL_IDX);
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(this->getLogger(), "Successfully initialized!");
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
    this->port_handler_.get());
  this->velocity_commander_ =
    std::make_unique<su065d4380_interface::VelocityCommander>(packet_handler_);
  this->info_commander_ =
    std::make_unique<su065d4380_interface::InfoCommander>(packet_handler_);

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
  // STOP motor
  this->velocity_commander_->writeVelocity(
    su065d4380_interface::FLAG_MODE_MOTOR_ON,
    0, 0);
  this->port_handler_->closePort();
  this->packet_handler_ = nullptr;
  this->velocity_commander_ = nullptr;
  this->info_commander_ = nullptr;
  RCLCPP_INFO(this->getLogger(), "Successfully deactivated!");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type SU065D4380System::read()
{
  while (this->packet_handler_->getBytesAvailable()) {
    this->packet_handler_->readPortIntoQueue();
  }
  static SU065D4380_RESPONSE_STATE vel_response;
  if (!this->last_command_accepted_) {
    vel_response = this->velocity_commander_->evaluateResponse();
    if (vel_response == SU065D4380_RESPONSE_STATE::OK) {
      RCLCPP_DEBUG(
        this->getLogger(),
        "Last command accepted!");
      this->last_command_accepted_ = true;
    } else if (vel_response == SU065D4380_RESPONSE_STATE::WAITING_RESPONSE) {
      RCLCPP_DEBUG(
        this->getLogger(), "Waiting for response");
    } else {
      su065d4380_interface::CommandUtil::logResponse(
        this->getLogger(), vel_response);
      return hardware_interface::return_type::ERROR;
    }
  }
  this->info_commander_->evaluateResponse();

  static const double RPM2RPS = (2.0 * M_PI) / 60.0;
  static uint8_t right_mode, left_mode;
  static SU065D4380_RESPONSE_STATE
    right_response, left_response, enc_response;
  static int16_t right_rpm, left_rpm;
  static uint16_t right_enc, left_enc;

  right_response = this->info_commander_->readRightRpm(right_mode, right_rpm);
  if (right_response == SU065D4380_RESPONSE_STATE::OK) {
    this->hw_velocities_.at(RIGHT_WHEEL_IDX) =
      static_cast<double>(right_rpm) * RPM2RPS;
  } else if (right_response == SU065D4380_RESPONSE_STATE::WAITING_RESPONSE) {
    RCLCPP_DEBUG(
      this->getLogger(), "Waiting for response");
  } else {
    su065d4380_interface::CommandUtil::logResponse(
      this->getLogger(), right_response);
    return hardware_interface::return_type::ERROR;
  }

  left_response = this->info_commander_->readLeftRpm(left_mode, left_rpm);
  if (left_response == SU065D4380_RESPONSE_STATE::OK) {
    this->hw_velocities_.at(LEFT_WHEEL_IDX) =
      static_cast<double>(left_rpm) * RPM2RPS;
  } else if (left_response == SU065D4380_RESPONSE_STATE::WAITING_RESPONSE) {
    RCLCPP_DEBUG(
      this->getLogger(), "Waiting for response");
  } else {
    su065d4380_interface::CommandUtil::logResponse(
      this->getLogger(), left_response);
    return hardware_interface::return_type::ERROR;
  }

  enc_response = this->info_commander_->readEncoderData(right_enc, left_enc);
  if (enc_response == su065d4380_interface::RESPONSE_STATE::OK) {
    // TODO(m12watanabe1a) make it absolute encoder
    this->hw_positions_.at(RIGHT_WHEEL_IDX) =
      static_cast<double>(right_enc);
    this->hw_positions_.at(LEFT_WHEEL_IDX) =
      static_cast<double>(left_enc);
  } else if (enc_response == SU065D4380_RESPONSE_STATE::WAITING_RESPONSE) {
    RCLCPP_DEBUG(
      this->getLogger(), "Waiting for response");
  } else {
    su065d4380_interface::CommandUtil::logResponse(
      this->getLogger(), enc_response);
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SU065D4380System::write()
{
  if (!this->last_command_accepted_) {
    return hardware_interface::return_type::ERROR;
  }

  static const double RPS2RPM = 60.0 / (2.0 * M_PI);
  const double left_rps =
    this->left_coefficient_ *
    this->hw_commands_.at(LEFT_WHEEL_IDX);
  const double right_rps =
    this->right_coefficient_ *
    this->hw_commands_.at(RIGHT_WHEEL_IDX);

  static SU065D4380_RESPONSE_STATE response;
  if (this->last_command_accepted_) {
    int16_t right_rpm = static_cast<int16_t>(right_rps * RPS2RPM);
    int16_t left_rpm = static_cast<int16_t>(left_rps * RPS2RPM);
    response = this->velocity_commander_->writeVelocity(
      su065d4380_interface::FLAG_MODE_MOTOR_ON, right_rpm, left_rpm);
    this->last_command_accepted_ = false;
  }
  hardware_interface::return_type ret = hardware_interface::return_type::ERROR;
  if (response == SU065D4380_RESPONSE_STATE::OK) {
    ret = hardware_interface::return_type::OK;
  } else if (response == SU065D4380_RESPONSE_STATE::WAITING_RESPONSE) {
    RCLCPP_DEBUG(
      this->getLogger(),
      "Waiting for response");
    ret = hardware_interface::return_type::OK;
  } else {
    su065d4380_interface::CommandUtil::logResponse(
      this->getLogger(),
      response);
  }
  return ret;
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
