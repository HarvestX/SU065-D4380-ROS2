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

#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "su065d4380_interface/packet_handler.hpp"

namespace su065d4380_interface
{
using namespace std::chrono_literals;
class VelocityCommander
{
public:
  static const int16_t MAX_RPM = 3000;

private:
  std::shared_ptr<PacketHandler> packet_handler_;
  rclcpp::Clock::SharedPtr clock_;
  const rclcpp::Duration TIMEOUT_;

  rclcpp::Time packet_sent_time_;

public:
  VelocityCommander() = delete;
  explicit VelocityCommander(
    std::shared_ptr<PacketHandler>,
    const std::chrono::nanoseconds = 1s);

  RESPONSE_STATE writeVelocity(
    const uint8_t,
    const int16_t,
    const int16_t) noexcept;
  RESPONSE_STATE evaluateResponse() const noexcept;

private:
  static const rclcpp::Logger getLogger();
};
}  // namespace su065d4380_interface
