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

#include "su065d4380_interface/commander/velocity_commander.hpp"

namespace su065d4380_interface
{
VelocityCommander::VelocityCommander(
  std::shared_ptr<PacketHandler> packet_handler,
  const std::chrono::nanoseconds timeout)
: packet_handler_(packet_handler),
  clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  TIMEOUT_(rclcpp::Duration(timeout))
{
}

RESPONSE_STATE VelocityCommander::writeVelocity(
  const uint16_t mode,
  const int32_t right_rpm,
  const int32_t left_rpm
) noexcept
{
  if (std::abs(right_rpm) > this->MAX_RPM) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Input out of range -3000 ~ 3000 [%d]",
      right_rpm);
    return RESPONSE_STATE::ERROR_INVALID_INPUT;
  } else if (std::abs(left_rpm) > this->MAX_RPM) {
    RCLCPP_ERROR(
      this->getLogger(),
      "Input out of range -3000 ~ 3000 [%d]",
      left_rpm);
    return RESPONSE_STATE::ERROR_INVALID_INPUT;
  }
  char write_buf[100];

  int cx = snprintf(
    write_buf, sizeof(write_buf),
    "$8C%02hhX00%04hX%04hX", mode, right_rpm, left_rpm);

  cx = CommandUtil::setChecksum(write_buf, sizeof(write_buf), cx);

  this->packet_handler_->writePort(write_buf, cx);
  this->packet_sent_time_ = this->clock_->now();

  return RESPONSE_STATE::WAITING_RESPONSE;
}

RESPONSE_STATE VelocityCommander::evaluateResponse() const noexcept
{
  static char const * const WRITE_OK = "$8C06\r";
  static char const * const WRITE_NG = "$8C07\r";

  std::string response;
  bool has_response = this->packet_handler_->takeVelocityPacket(response);
  if (!has_response) {
    if (this->clock_->now() - this->packet_sent_time_ > TIMEOUT_) {
      return RESPONSE_STATE::ERROR_NOT_COMING_YET;
    } else {
      return RESPONSE_STATE::WAITING_RESPONSE;
    }
  } else if (response == WRITE_NG) {
    return RESPONSE_STATE::ERROR_EXPLICIT_NG;
  } else if (response == WRITE_OK) {
    return RESPONSE_STATE::OK;
  }

  return RESPONSE_STATE::ERROR_UNKNOWN;
}

const rclcpp::Logger VelocityCommander::getLogger()
{
  return rclcpp::get_logger("VelocityCommander");
}
}  // namespace su065d4380_interface
