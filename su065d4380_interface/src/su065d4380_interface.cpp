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

#include "su065d4380_interface/su065d4380_interface.hpp"

namespace su065d4380_interface
{

SU065D4380Interface::SU065D4380Interface(
  const std::string & _port_name,
  const std::chrono::nanoseconds _timeout)
: clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  TIMEOUT_(rclcpp::Duration(_timeout))
{
  this->port_handler_ =
    std::make_unique<PortHandler>(_port_name);
}

bool SU065D4380Interface::init()
{
  // Dry run to check can open port or not
  if (!this->port_handler_->openPort()) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Failed to open port");
    return false;
  }
  this->port_handler_->closePort();
  return true;
}

bool SU065D4380Interface::activate()
{
  if (!this->port_handler_->openPort()) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Failed to open port");
    return false;
  }

  this->packet_handler_ =
    std::make_unique<PacketHandler>(
    this->port_handler_.get());

  this->velocity_commander_ =
    std::make_unique<VelocityCommander>(
    this->packet_handler_,
    this->TIMEOUT_);
  this->info_commander_ =
    std::make_unique<InfoCommander>(
    this->packet_handler_,
    this->TIMEOUT_);

  this->last_velocity_command_accepted = true;

  return true;
}

bool SU065D4380Interface::deactivate()
{
  // Stop motor
  this->velocity_commander_->writeVelocity(
    su065d4380_interface::FLAG_MODE_MOTOR_ON, 0, 0);

  this->port_handler_->closePort();
  this->packet_handler_ = nullptr;

  this->velocity_commander_ = nullptr;
  this->info_commander_ = nullptr;

  return true;
}

bool SU065D4380Interface::readPreprocess() const noexcept
{
  rclcpp::Time time_started = this->clock_->now();
  while (this->packet_handler_->getBytesAvailable()) {
    this->packet_handler_->readPortIntoQueue();
    if (this->clock_->now() - time_started > TIMEOUT_) {
      return false;
    }
  }
  this->info_commander_->evaluateResponse();
  return true;
}

bool SU065D4380Interface::readLastVelocityCommandState() noexcept
{
  static RESPONSE_STATE response;
  if (this->last_velocity_command_accepted) {
    return true;
  }

  response = this->velocity_commander_->evaluateResponse();
  bool res = this->processResponse(response);
  this->last_velocity_command_accepted = res;
  return res;
}

bool SU065D4380Interface::readRightRpm(
  int16_t & right_rpm) const noexcept
{
  static uint8_t mode;
  static RESPONSE_STATE response;
  response = this->info_commander_->readRightRpm(mode, right_rpm);
  return this->processResponse(response);
}

bool SU065D4380Interface::readLeftRpm(
  int16_t & left_rpm) const noexcept
{
  static uint8_t mode;
  static RESPONSE_STATE response;
  response = this->info_commander_->readLeftRpm(mode, left_rpm);
  return this->processResponse(response);
}

bool SU065D4380Interface::readEncoder(
  uint16_t & right_enc, uint16_t & left_enc) const noexcept
{
  static RESPONSE_STATE response;
  response = this->info_commander_->readEncoderData(right_enc, left_enc);
  return this->processResponse(response);
}

bool SU065D4380Interface::writeVelocity(
  const int16_t & right_rpm, const int16_t & left_rpm) noexcept
{
  if (!this->last_velocity_command_accepted) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Waiting previous command accept");
    return false;
  }

  static RESPONSE_STATE response;
  response = this->velocity_commander_->writeVelocity(
    FLAG_MODE_MOTOR_ON, right_rpm, left_rpm);
  this->last_velocity_command_accepted = false;
  if (response == RESPONSE_STATE::ERROR_INVALID_INPUT) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Invalid Input Given right: %d [rpm] left: %d [rpm]",
      right_rpm, left_rpm);
    // Return true to don't stop the system
    return true;
  }

  return this->processResponse(response);
}


const rclcpp::Logger getLogger() noexcept
{
  return rclcpp::get_logger("SU065D4380Interface");
}

bool SU065D4380Interface::processResponse(
  const RESPONSE_STATE & response) noexcept
{
  bool ret;
  switch (response) {
    case RESPONSE_STATE::OK:
      ret = true;
      break;
    case RESPONSE_STATE::WAITING_RESPONSE:
      RCLCPP_DEBUG(
        SU065D4380Interface::getLogger(),
        "Waiting for response");
      ret = true;
      break;
    case RESPONSE_STATE::ERROR_EXPLICIT_NG:
    case RESPONSE_STATE::ERROR_NOT_COMING_YET:
    case RESPONSE_STATE::ERROR_INVALID_INPUT:
    case RESPONSE_STATE::ERROR_CRC:
    case RESPONSE_STATE::ERROR_UNKNOWN:
      CommandUtil::logResponse(
        SU065D4380Interface::getLogger(),
        response);
      ret = false;
      break;
    default:
      ret = false;
      break;
  }
  return ret;
}


}  // namespace su065d4380_interface
