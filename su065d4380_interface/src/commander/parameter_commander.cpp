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
  PortHandlerBase * port_handler,
  const std::chrono::nanoseconds timeout
)
: port_handler_(port_handler),
  clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  TIMEOUT(rclcpp::Duration(timeout)),
  pool_(std::make_shared<PacketPool>())
{
}

bool ParameterCommander::writeRightWheelGain(
  const uint gain)
{
  if (gain > this->MAX_GAIN_) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Input out of range 0 ~ 100 [%u]", gain);
    return false;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00W001E%04hX", gain);

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateWriteResponse();
}

bool ParameterCommander::writeLeftWheelGain(
  const uint gain)
{
  if (gain > this->MAX_GAIN_) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Input out of range 0 ~ 100 [%u]", gain);
    return false;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00W001F%04hX", gain);

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateWriteResponse();
}

bool ParameterCommander::writeAccTime(const uint time)
{
  if (time > this->MAX_ACCTIME_) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Input out of range 0 ~ 500 [%u]", time);
    return false;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00W0020%04hX", time);

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateWriteResponse();
}

bool ParameterCommander::writeDecTime(const uint time)
{
  if (time > this->MAX_ACCTIME_) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Input out of range 0 ~ 500 [%u]", time);
    return false;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00W0021%04hX", time);

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateWriteResponse();
}

bool ParameterCommander::writeTimeout(const uint time)
{
  if (time > this->MAX_TIMEOUT_) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Input out of range 0 ~ 5 [%u]", time);
    return false;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00W0025%04hX", time);

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateWriteResponse();
}

bool ParameterCommander::writeDecWithTimeout(const uint time)
{
  if (time > this->MAX_ACCTIME_) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Input out of range 0 ~ 500 [%u]", time);
    return false;
  }
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00W0026%04hX", time);

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateWriteResponse();
}

int ParameterCommander::readRightWheelGain()
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00R001E");

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateReadResponse();
}

int ParameterCommander::readLeftWheelGain()
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00R001F");

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateReadResponse();
}

int ParameterCommander::readAccTime()
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00R0020");

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateReadResponse();
}

int ParameterCommander::readDecTime()
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00R0021");

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateReadResponse();
}

int ParameterCommander::readTimeout()
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00R0025");

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateReadResponse();
}

int ParameterCommander::readDecWithTimeout()
{
  char write_buf[100];

  // Set base string
  int cx = snprintf(
    write_buf, sizeof(write_buf), "$00R0026");

  cx = this->setChecksum(write_buf, sizeof(write_buf), cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateReadResponse();
}

uint16_t ParameterCommander::calcChecksum(
  char const * const buf, const int cx) const
{
  uint16_t crc = 0;
  for (int i = 0; i < cx; ++i) {
    crc ^= static_cast<uint16_t>(buf[i]);
  }

  return crc;
}

int ParameterCommander::setChecksum(
  char * const buf, const size_t buf_size, const int cx)
{
  const uint16_t crc = this->calcChecksum(buf, cx);

  return snprintf(
    buf + cx, buf_size - cx, "%02hhX\r", crc) + cx;
}

bool ParameterCommander::waitForResponse(std::string & response)
{
  bool has_response = false;
  const auto start = this->clock_->now();
  while (this->clock_->now() - start < this->TIMEOUT) {
    if (this->port_handler_->getBytesAvailable() < 1) {
      continue;
    }
    char buf[100];
    this->port_handler_->readPort(buf, sizeof(buf));
    this->pool_->enqueue(std::string(buf));
    if (this->pool_->takeParamPacket(response)) {
      has_response = true;
      break;
    }
    rclcpp::sleep_for(100ms);
  }
  return has_response;
}

bool ParameterCommander::evaluateWriteResponse()
{
  static const char * const WRITE_OK = "$00W17*\r";
  static const char * const WRITE_NG = "$00W17/\r";

  std::string response;
  bool has_response = this->waitForResponse(response);
  if (!has_response) {
    throw std::runtime_error("Couldn't get response before timeout");
  }
  if (response == WRITE_OK) {
    return true;
  } else if (response == WRITE_NG) {
    return false;
  } else {
    throw std::runtime_error("Invalid response");
  }
}

int ParameterCommander::evaluateReadResponse()
{
  static const int READ_DATA_IDX = 4;
  static const int READ_CHECKSUM_IDX = 8;
  static const char * const READ_NG = "$00R12/\r";

  std::string response;
  const bool has_response = this->waitForResponse(response);
  if (!has_response) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Couldn't get response before timeout");
    return -1;
  } else if (response == READ_NG) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Driver responded NG");
    return -1;
  }

  // Check crc
  uint16_t expected_crc;
  try {
    expected_crc =
      std::stoi(response.substr(READ_CHECKSUM_IDX, 2), nullptr, 16);
  } catch (std::invalid_argument &) {
    return false;
  }
  const uint16_t actual_crc =
    this->calcChecksum(response.data(), READ_CHECKSUM_IDX);
  if (expected_crc != actual_crc) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Crc does not match [%u] != [%u]",
      expected_crc, actual_crc);
    return -1;
  }

  // Fine result
  return std::stoi(response.substr(READ_DATA_IDX, 4), nullptr, 16);
}

const rclcpp::Logger ParameterCommander::getLogger()
{
  return rclcpp::get_logger("ParameterCommander");
}
}  // namespace su065d4380_interface
