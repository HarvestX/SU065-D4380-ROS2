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
using namespace std::chrono_literals;  // NOLINT
class ParameterCommander
{
public:
  using SharedPtr = std::shared_ptr<ParameterCommander>;
  using UniquePtr = std::unique_ptr<ParameterCommander>;

  static const uint MAX_GAIN = 100;
  static const uint MAX_ACCTIME = 500;
  static const uint MAX_TIMEOUT = 5;

private:
  PacketHandler::SharedPtr packet_handler_;
  rclcpp::Clock::SharedPtr clock_;
  const rclcpp::Duration TIMEOUT_;

public:
  ParameterCommander() = delete;
  explicit ParameterCommander(PacketHandler::SharedPtr, const rclcpp::Duration &);

  // Write parameter
  RESPONSE_STATE writeRightWheelGain(const uint) const noexcept;
  RESPONSE_STATE writeLeftWheelGain(const uint) const noexcept;
  RESPONSE_STATE writeAccTime(const uint) const noexcept;
  RESPONSE_STATE writeDecTime(const uint) const noexcept;
  RESPONSE_STATE writeTimeout(const uint) const noexcept;
  RESPONSE_STATE writeDecWithTimeout(const uint) const noexcept;

  // Read parameter
  RESPONSE_STATE readRightWheelGain(int &) const noexcept;
  RESPONSE_STATE readLeftWheelGain(int &) const noexcept;
  RESPONSE_STATE readAccTime(int &) const noexcept;
  RESPONSE_STATE readDecTime(int &) const noexcept;
  RESPONSE_STATE readTimeout(int &) const noexcept;
  RESPONSE_STATE readDecWithTimeout(int &) const noexcept;

private:
  static const rclcpp::Logger getLogger();
  bool waitForResponse(std::string &) const noexcept;
  RESPONSE_STATE evaluateWriteResponse() const noexcept;
  RESPONSE_STATE evaluateReadResponse(int &)const noexcept;
};
}  // namespace su065d4380_interface
