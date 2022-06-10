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
#include <queue>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "su065d4380_interface/port_handler.hpp"

#include "su065d4380_interface/packet/velocity_packet.hpp"
#include "su065d4380_interface/packet/info_packet.hpp"

namespace su065d4380_interface
{

class PacketHandler
{
private:
  const rclcpp::Logger logger_ = rclcpp::get_logger("PacketHandler");
  const std::shared_ptr<PortHandler> port_handler_;

  const std::unique_ptr<std::queue<std::string>> queue_vel_rx;
  const std::unique_ptr<std::queue<std::string>> queue_inf_rx;

  const double ENC2RAD_ = 2.0 * M_PI / 65535;
  const double RPM2RPS_ = 60.0 / M_PI / 2.0;
  const double RPS2RPM_ = 1.0 / RPM2RPS_;

  std::unique_ptr<info_packet::DriverState> driver_state_;

  enum class VELCOM_STATE
  {
    WAITING_RESPONSE,
    READY,
  };

  VELCOM_STATE velcom_state_;

  float voltage_ = NAN;
  int right_rpm_ = 0;
  int left_rpm_ = 0;
  uint32_t right_encoder_ = 0;
  uint32_t left_encode_ = 0;

public:
  PacketHandler() = delete;
  explicit PacketHandler(std::shared_ptr<PortHandler>);

  bool sendVelocityCommand(const double, const double);
  bool sendVelocityCommand(const uint16_t, const int32_t, const int32_t);
  void recvCommand();

  double getVoltage();
  double getLeftPosition();
  double getRightPosition();
  double getLeftVelocity();
  double getRightVelocity();

  // TODO(m12watanabe1a): Implement it
  // bool sendConfigCommand();
  // bool recvConfigCommand();

private:
  void enqueueCommands(const std::string &);
  void evaluateCommands();
};
}  // namespace su065d4380_interface
