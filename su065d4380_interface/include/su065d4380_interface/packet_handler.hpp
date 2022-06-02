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

namespace su065d4380_interface
{

class PacketHandler
{
private:
  const rclcpp::Logger logger_ = rclcpp::get_logger("PacketHandler");
  const std::unique_ptr<PortHandler> port_handler_;

  const std::unique_ptr<std::queue<std::string>> queue_vel_rx;
  const std::unique_ptr<std::queue<std::string>> queue_inf_rx;

public:
  PacketHandler() = delete;
  explicit PacketHandler(std::unique_ptr<PortHandler>);

  bool sendVelocityCommand();
  void recvCommand();

  // TODO: Implement it
  // bool sendConfigCommand();
  // bool recvConfigCommand();
};
}  // namespace su065d4380_interface
