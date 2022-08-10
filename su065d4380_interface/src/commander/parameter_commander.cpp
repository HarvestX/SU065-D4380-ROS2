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
  char write_buf[this->BUF_SIZE_];

  // Set base string
  int cx = snprintf(
    write_buf, this->BUF_SIZE_, "$00W001E%04hX", gain);

  cx = this->setChecksum(write_buf, cx);

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
  char write_buf[this->BUF_SIZE_];

  // Set base string
  int cx = snprintf(
    write_buf, this->BUF_SIZE_, "$00W001F%04hX", gain);

  cx = this->setChecksum(write_buf, cx);

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
  char write_buf[this->BUF_SIZE_];

  // Set base string
  int cx = snprintf(
    write_buf, this->BUF_SIZE_, "$00W0020%04hX", time);

  cx = this->setChecksum(write_buf, cx);

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
  char write_buf[this->BUF_SIZE_];

  // Set base string
  int cx = snprintf(
    write_buf, this->BUF_SIZE_, "$00W0021%04hX", time);

  cx = this->setChecksum(write_buf, cx);

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
  char write_buf[this->BUF_SIZE_];

  // Set base string
  int cx = snprintf(
    write_buf, this->BUF_SIZE_, "$00W0025%04hX", time);

  cx = this->setChecksum(write_buf, cx);

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
  char write_buf[this->BUF_SIZE_];

  // Set base string
  int cx = snprintf(
    write_buf, this->BUF_SIZE_, "$00W0026%04hX", time);

  cx = this->setChecksum(write_buf, cx);

  // Send Command
  this->port_handler_->writePort(write_buf, cx);
  return this->evaluateWriteResponse();
}

int ParameterCommander::setChecksum(
  char * const buf, const int cx)
{
  uint16_t sum = 0;
  for (int i = 0; i < cx; ++i) {
    sum ^= static_cast<uint16_t>(buf[i]);
  }

  return snprintf(
    buf + cx, this->BUF_SIZE_ - cx, "%02hhX\r", sum);
}

bool ParameterCommander::evaluateWriteResponse()
{
  bool has_response = false;
  std::string response;
  const auto start = this->clock_->now();
  while (this->clock_->now() - start < this->TIMEOUT) {
    char buf[100];
    this->port_handler_->readPort(buf, 100);
    this->pool_->enqueue(std::string(buf));
    if (this->pool_->takeParamPacket(response)) {
      std::cout << response << std::endl;
      has_response = true;
      break;
    }
    rclcpp::sleep_for(10ms);
  }
  if (!has_response) {
    throw std::runtime_error("Couldn't take the response before the timeout.");
  }
  if (response == parameter_packet::WRITE_OK) {
    return true;
  } else if (response == parameter_packet::WRITE_NG) {
    return false;
  } else {
    throw std::runtime_error("Invalid response.");
  }
}

const rclcpp::Logger ParameterCommander::getLogger()
{
  return rclcpp::get_logger("ParameterCommander");
}
}  // namespace su065d4380_interface
