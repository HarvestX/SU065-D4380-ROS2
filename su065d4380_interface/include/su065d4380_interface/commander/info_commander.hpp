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

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "su065d4380_interface/packet_handler.hpp"

namespace su065d4380_interface
{
using namespace std::chrono_literals;

class InfoPacket
{
private:
  std::string packet_;
  bool is_updated_;
  rclcpp::Clock::SharedPtr clock_;
  const rclcpp::Duration TIMEOUT_;
  rclcpp::Time last_updated_time_;

public:
  InfoPacket() = delete;
  InfoPacket(rclcpp::Clock::SharedPtr, const rclcpp::Duration);
  void setPacket(const std::string &) noexcept;
  RESPONSE_STATE takePacket(std::string &)noexcept;
};

class DriverState
{
private:
  bool in_operation_;
  bool has_error_;

  VOLTAGE_STATE voltage_state_;
  ERROR_STATE error_state_;

public:
  bool hasError() const noexcept;
  bool inOperation() const noexcept;

  VOLTAGE_STATE getVoltageState() const noexcept;
  ERROR_STATE getErrorState() const noexcept;

  void setDriverState(const uint16_t &) noexcept;
  void setErrorState(const uint16_t &) noexcept;
};

class InfoCommander
{
public:
  enum class COMMAND_TYPE
  {
    RIGHT_WHEEL,
    LEFT_WHEEL,
    DRIVER_STATE,
    ENCODE_DATA,
    VOLTAGE,
    INVALID
  };

private:
  std::shared_ptr<PacketHandler> packet_handler_;
  rclcpp::Clock::SharedPtr clock_;
  const rclcpp::Duration TIMEOUT_;

  std::unique_ptr<InfoPacket> last_right_wheel_packet_;
  std::unique_ptr<InfoPacket> last_left_wheel_packet_;
  std::unique_ptr<InfoPacket> last_driver_state_packet_;
  std::unique_ptr<InfoPacket> last_encode_data_packet_;
  std::unique_ptr<InfoPacket> last_voltage_packet_;

public:
  InfoCommander() = delete;
  explicit InfoCommander(
    std::shared_ptr<PacketHandler>,
    const std::chrono::nanoseconds = 1s);

  RESPONSE_STATE readRightRpm(uint8_t &, int16_t &);
  RESPONSE_STATE readLeftRpm(uint8_t &, int16_t &);
  RESPONSE_STATE readDriverState(DriverState &);
  RESPONSE_STATE readEncoderData(int16_t &, int16_t &);
  RESPONSE_STATE readVoltage(float &);

  void evaluateResponse()  noexcept;

private:
  static const rclcpp::Logger getLogger();
  COMMAND_TYPE getCommandType(const std::string &) const noexcept;

};
}  // namespace su065d4380_interface
