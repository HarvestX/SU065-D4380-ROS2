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

#include "su065d4380_interface/commander/info_commander.hpp"

namespace su065d4380_interface
{
InfoPacket::InfoPacket(
  rclcpp::Clock::SharedPtr clock,
  const rclcpp::Duration timeout)
: is_updated_(false),
  clock_(clock),
  TIMEOUT_(timeout)
{
  this->last_updated_time_ = this->clock_->now();
}

void InfoPacket::setPacket(const std::string & packet) noexcept
{
  this->is_updated_ = true;
  this->last_updated_time_ = this->clock_->now();
  this->packet_ = packet;
}

RESPONSE_STATE InfoPacket::takePacket(std::string & packet) noexcept
{
  if (!this->is_updated_) {
    if (this->clock_->now() - this->last_updated_time_ > this->TIMEOUT_) {
      return RESPONSE_STATE::ERROR_NOT_COMING_YET;
    } else {
      return RESPONSE_STATE::WAITING_RESPONSE;
    }
  }
  packet = this->packet_;
  this->is_updated_ = false;
  return RESPONSE_STATE::OK;
}

bool DriverState::hasError() const noexcept
{
  return this->has_error_;
}
bool DriverState::inOperation() const noexcept
{
  return this->in_operation_;
}

DriverState::VOLTAGE_STATE DriverState::getVoltageState() const noexcept
{
  return this->voltage_state_;
}

DriverState::ERROR_STATE DriverState::getErrorState() const noexcept
{
  return this->error_state_;
}

void DriverState::setDriverState(const uint16_t & driver_state) noexcept
{
  this->in_operation_ =
    (driver_state & (1 << 0));
  this->has_error_ =
    (driver_state & (1 << 1));

  if (driver_state & (1 << 2)) {
    this->voltage_state_ = VOLTAGE_STATE::LOW_WARNING;
  } else if (driver_state & (1 << 3)) {
    this->voltage_state_ = VOLTAGE_STATE::LOW;
  } else if (driver_state & (1 << 4)) {
    this->voltage_state_ = VOLTAGE_STATE::LOW_EMERGENCY;
  } else {
    this->voltage_state_ = VOLTAGE_STATE::OK;
  }
}

void DriverState::setErrorState(const uint16_t & error_state) noexcept
{
  if (error_state & (1 << 0)) {
    this->error_state_ = DriverState::ERROR_STATE::LOW_VOLTAGE;
  } else if (error_state & (1 << 1)) {
    this->error_state_ = DriverState::ERROR_STATE::HIGH_VOLTAGE;
  } else if (error_state & (1 << 2)) {
    this->error_state_ = DriverState::ERROR_STATE::INTERNAL_DRIVER_ERROR;
  } else if (error_state & (1 << 3)) {
    this->error_state_ = DriverState::ERROR_STATE::SENSOR_ERROR;
  } else if (error_state & (1 << 4)) {
    this->error_state_ = DriverState::ERROR_STATE::OVER_CURRENT;
  } else if (error_state & (1 << 5)) {
    this->error_state_ = DriverState::ERROR_STATE::INVALID_VELOCITY;
  } else if (error_state & (1 << 6)) {
    this->error_state_ = DriverState::ERROR_STATE::OVER_LOAD;
  } else if (error_state & (1 << 7)) {
    this->error_state_ = DriverState::ERROR_STATE::COMMUNICATION_ERROR;
  } else {
    this->error_state_ = DriverState::ERROR_STATE::OK;
  }
}

InfoCommander::InfoCommander(
  std::shared_ptr<PacketHandler> packet_handler,
  const std::chrono::nanoseconds timeout
)
: packet_handler_(packet_handler),
  clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  TIMEOUT_(rclcpp::Duration(timeout))
{
  this->last_right_wheel_packet_ =
    std::make_unique<InfoPacket>(this->clock_, this->TIMEOUT_);
  this->last_left_wheel_packet_ =
    std::make_unique<InfoPacket>(this->clock_, this->TIMEOUT_);
  this->last_driver_state_packet_ =
    std::make_unique<InfoPacket>(this->clock_, this->TIMEOUT_);
  this->last_encode_data_packet_ =
    std::make_unique<InfoPacket>(this->clock_, this->TIMEOUT_);
  this->last_voltage_packet_ =
    std::make_unique<InfoPacket>(this->clock_, this->TIMEOUT_);
}

RESPONSE_STATE InfoCommander::readRightRpm(
  uint8_t & mode, int16_t & rpm)
{
  std::string packet;
  const RESPONSE_STATE state =
    this->last_right_wheel_packet_->takePacket(packet);
  if (state != RESPONSE_STATE::OK) {
    return state;
  }

  static const size_t MODE_IDX = 3;
  static const size_t RPM_IDX = 7;
  try {
    mode =
      static_cast<uint8_t>(std::stoi(packet.substr(MODE_IDX, 2), nullptr, 16));
    rpm =
      static_cast<int16_t>(std::stoi(packet.substr(RPM_IDX, 4), nullptr, 16));
  } catch (std::invalid_argument &) {
    return RESPONSE_STATE::ERROR_UNKNOWN;
  }

  return RESPONSE_STATE::OK;
}

RESPONSE_STATE InfoCommander::readLeftRpm(
  uint8_t & mode, int16_t & rpm)
{
  std::string packet;
  const RESPONSE_STATE state =
    this->last_left_wheel_packet_->takePacket(packet);
  if (state != RESPONSE_STATE::OK) {
    return state;
  }

  static const size_t MODE_IDX = 3;
  static const size_t RPM_IDX = 7;
  try {
    mode =
      static_cast<uint8_t>(std::stoi(packet.substr(MODE_IDX, 2), nullptr, 16));
    rpm =
      static_cast<int16_t>(std::stoi(packet.substr(RPM_IDX, 4), nullptr, 16));
  } catch (std::invalid_argument &) {
    return RESPONSE_STATE::ERROR_UNKNOWN;
  }

  return RESPONSE_STATE::OK;
}

RESPONSE_STATE InfoCommander::readDriverState(DriverState & _driver_state)
{
  std::string packet;
  const RESPONSE_STATE state =
    this->last_driver_state_packet_->takePacket(packet);
  if (state != RESPONSE_STATE::OK) {
    return state;
  }

  static const size_t DRIVER_STATE_IDX = 3;
  static const size_t ERROR_STATE_IDX = 7;

  try {
    const uint16_t driver_state =
      std::stoi(packet.substr(DRIVER_STATE_IDX, 4), nullptr, 16);

    _driver_state.setDriverState(driver_state);

    if (_driver_state.hasError()) {
      const uint16_t error_state =
        std::stoi(packet.substr(ERROR_STATE_IDX, 4), nullptr, 16);
      _driver_state.setErrorState(error_state);
    }
  } catch (std::invalid_argument &) {
    return RESPONSE_STATE::ERROR_UNKNOWN;
  }
  return RESPONSE_STATE::OK;
}

RESPONSE_STATE InfoCommander::readEncoderData(
  uint16_t & right_encoder, uint16_t & left_encoder)
{
  std::string packet;
  const RESPONSE_STATE state =
    this->last_encode_data_packet_->takePacket(packet);
  if (state != RESPONSE_STATE::OK) {
    return state;
  }

  static const size_t RIGHT_ENCODER_IDX = 3;
  static const size_t LEFT_ENCODER_IDX = 7;
  try {
    right_encoder =
      static_cast<uint16_t>(
      std::stoi(packet.substr(RIGHT_ENCODER_IDX, 4), nullptr, 16));
    left_encoder =
      static_cast<uint16_t>(
      std::stoi(packet.substr(LEFT_ENCODER_IDX, 4), nullptr, 16));
  } catch (std::invalid_argument &) {
    return RESPONSE_STATE::ERROR_UNKNOWN;
  }

  return RESPONSE_STATE::OK;
}

RESPONSE_STATE InfoCommander::readVoltage(float & voltage)
{
  std::string packet;
  const RESPONSE_STATE state =
    this->last_voltage_packet_->takePacket(packet);
  if (state != RESPONSE_STATE::OK) {
    return state;
  }

  static const size_t VOLTAGE_IDX = 3;
  try {
    voltage = static_cast<float>(
      std::stoi(packet.substr(VOLTAGE_IDX, 4), nullptr, 16)) * 1e-2;
  } catch (std::invalid_argument &) {
    return RESPONSE_STATE::ERROR_UNKNOWN;
  }

  return RESPONSE_STATE::OK;
}

/**
 * @brief Evaluate the response from read packet
 * @note Call PacketHandler enqueue method before get the result
 *
 * @return RESPONSE_STATE
 */
void InfoCommander::evaluateResponse() noexcept
{
  static const size_t CHECKSUM_IDX = 11;
  std::string response;
  while (this->packet_handler_->takeInfoPacket(response)) {
    if (!CommandUtil::confirmChecksum(response, CHECKSUM_IDX)) {
      continue;
    }

    const InfoCommander::COMMAND_TYPE command_type =
      this->getCommandType(response);

    switch (command_type) {
      case InfoCommander::COMMAND_TYPE::RIGHT_WHEEL:
        this->last_right_wheel_packet_->setPacket(response);
        break;
      case InfoCommander::COMMAND_TYPE::LEFT_WHEEL:
        this->last_left_wheel_packet_->setPacket(response);
        break;
      case InfoCommander::COMMAND_TYPE::DRIVER_STATE:
        this->last_driver_state_packet_->setPacket(response);
        break;
      case InfoCommander::COMMAND_TYPE::ENCODE_DATA:
        this->last_encode_data_packet_->setPacket(response);
        break;
      case InfoCommander::COMMAND_TYPE::VOLTAGE:
        this->last_voltage_packet_->setPacket(response);
        break;
      case InfoCommander::COMMAND_TYPE::INVALID:
      default:
        continue;
    }
  }
}


const rclcpp::Logger InfoCommander::getLogger()
{
  return rclcpp::get_logger("InfoCommander");
}

InfoCommander::COMMAND_TYPE InfoCommander::getCommandType(
  const std::string & packet) const noexcept
{
  static const size_t COMMAND_IDX = 1;
  const std::string command_id = packet.substr(COMMAND_IDX, 2);

  if (command_id == "A1") {
    return COMMAND_TYPE::RIGHT_WHEEL;
  } else if (command_id == "A2") {
    return COMMAND_TYPE::LEFT_WHEEL;
  } else if (command_id == "A3") {
    return COMMAND_TYPE::DRIVER_STATE;
  } else if (command_id == "A4") {
    return COMMAND_TYPE::ENCODE_DATA;
  } else if (command_id == "A5") {
    return COMMAND_TYPE::VOLTAGE;
  }
  return COMMAND_TYPE::INVALID;
}
}  // namespace su065d4380_interface
