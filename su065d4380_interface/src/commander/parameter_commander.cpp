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

#include "su065d4380_interface/commander/parameter_commander.hpp"

namespace su065d4380_interface
{
ParameterCommander::ParameterCommander(
  std::shared_ptr<PacketHandler> packet_handler, const rclcpp::Duration & timeout)
: packet_handler_(packet_handler),
  clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  TIMEOUT_(timeout)
{}

RESPONSE_STATE ParameterCommander::writeRightWheelGain(const uint gain) const noexcept
{
  if (gain > this->MAX_GAIN) {
    RCLCPP_ERROR(this->getLogger(), "Input out of range 0 ~ 100 [%u]", gain);
    return RESPONSE_STATE::ERROR_INVALID_INPUT;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(write_buf, sizeof(write_buf), "$00W001E%04hX", gain);

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateWriteResponse();
}

RESPONSE_STATE ParameterCommander::writeLeftWheelGain(
  const uint gain) const noexcept
{
  if (gain > this->MAX_GAIN) {
    RCLCPP_ERROR(this->getLogger(), "Input out of range 0 ~ 100 [%u]", gain);
    return RESPONSE_STATE::ERROR_INVALID_INPUT;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(write_buf, sizeof(write_buf), "$00W001F%04hX", gain);

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateWriteResponse();
}

RESPONSE_STATE ParameterCommander::writeAccTime(const uint time) const noexcept
{
  if (time > this->MAX_ACCTIME) {
    RCLCPP_ERROR(this->getLogger(), "Input out of range 0 ~ 500 [%u]", time);
    return RESPONSE_STATE::ERROR_INVALID_INPUT;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00W0020%04hX", time);

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateWriteResponse();
}

RESPONSE_STATE ParameterCommander::writeDecTime(const uint time) const noexcept
{
  if (time > this->MAX_ACCTIME) {
    RCLCPP_ERROR(this->getLogger(), "Input out of range 0 ~ 500 [%u]", time);
    return RESPONSE_STATE::ERROR_INVALID_INPUT;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00W0021%04hX", time);

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateWriteResponse();
}

RESPONSE_STATE ParameterCommander::writeTimeout(const uint time) const noexcept
{
  if (time > this->MAX_TIMEOUT) {
    RCLCPP_ERROR(this->getLogger(), "Input out of range 0 ~ 5 [%u]", time);
    return RESPONSE_STATE::ERROR_INVALID_INPUT;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00W0025%04hX", time);

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateWriteResponse();
}

RESPONSE_STATE ParameterCommander::writeDecWithTimeout(const uint time) const noexcept
{
  if (time > this->MAX_ACCTIME) {
    RCLCPP_ERROR(this->getLogger(), "Input out of range 0 ~ 500 [%u]", time);
    return RESPONSE_STATE::ERROR_INVALID_INPUT;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(write_buf, sizeof(write_buf), "$00W0026%04hX", time);

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateWriteResponse();
}

RESPONSE_STATE ParameterCommander::readRightWheelGain(int & out) const noexcept
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(write_buf, sizeof(write_buf), "$00R001E");

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateReadResponse(out);
}

RESPONSE_STATE ParameterCommander::readLeftWheelGain(int & out) const noexcept
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(write_buf, sizeof(write_buf), "$00R001F");

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateReadResponse(out);
}

RESPONSE_STATE ParameterCommander::readAccTime(int & out) const noexcept
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(write_buf, sizeof(write_buf), "$00R0020");

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateReadResponse(out);
}

RESPONSE_STATE ParameterCommander::readDecTime(int & out) const noexcept
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(write_buf, sizeof(write_buf), "$00R0021");

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateReadResponse(out);
}

RESPONSE_STATE ParameterCommander::readTimeout(int & out) const noexcept
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(write_buf, sizeof(write_buf), "$00R0025");

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateReadResponse(out);
}

RESPONSE_STATE ParameterCommander::readDecWithTimeout(int & out) const noexcept
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(write_buf, sizeof(write_buf), "$00R0026");

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  if (this->packet_handler_->writePort(write_buf, cx) == -1) {
    return RESPONSE_STATE::ERROR_SENDING;
  }
  return this->evaluateReadResponse(out);
}

bool ParameterCommander::waitForResponse(std::string & response) const noexcept
{
  bool has_response = false;
  const auto start = this->clock_->now();
  while (this->clock_->now() - start < this->TIMEOUT_) {
    if (this->packet_handler_->getBytesAvailable() < 1) {
      rclcpp::sleep_for(10ms);
      continue;
    }
    this->packet_handler_->readPortIntoQueue();
    if (this->packet_handler_->takePacket(PacketPool::PACKET_TYPE::PARAM, response)) {
      has_response = true;
      break;
    }
    rclcpp::sleep_for(10ms);
  }
  return has_response;
}

RESPONSE_STATE ParameterCommander::evaluateWriteResponse() const noexcept
{
  static const char * const WRITE_OK = "$00W17*\r";
  static const char * const WRITE_NG = "$00W17/\r";

  std::string response;
  bool has_response = this->waitForResponse(response);
  if (!has_response) {
    return RESPONSE_STATE::ERROR_NOT_COMING_YET;
  }
  if (response == WRITE_OK) {
    return RESPONSE_STATE::OK;
  } else if (response == WRITE_NG) {
    return RESPONSE_STATE::ERROR_EXPLICIT_NG;
  }

  return RESPONSE_STATE::ERROR_UNKNOWN;
}

RESPONSE_STATE ParameterCommander::evaluateReadResponse(int & out) const noexcept
{
  static const int READ_DATA_IDX = 4;
  static const int READ_CHECKSUM_IDX = 8;
  static const char * const READ_NG = "$00R12/\r";

  std::string response;
  const bool has_response = this->waitForResponse(response);
  if (!has_response) {
    return RESPONSE_STATE::ERROR_NOT_COMING_YET;
  } else if (response == READ_NG) {
    return RESPONSE_STATE::ERROR_EXPLICIT_NG;
  }

  if (!CommandUtil::confirmChecksum(response, READ_CHECKSUM_IDX)) {
    return RESPONSE_STATE::ERROR_CRC;
  }

  // Fine result
  out = std::stoi(response.substr(READ_DATA_IDX, 4), nullptr, 16);
  return RESPONSE_STATE::OK;
}

const rclcpp::Logger ParameterCommander::getLogger()
{
  return rclcpp::get_logger("ParameterCommander");
}
}  // namespace su065d4380_interface
