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

#include <map>
#include <memory>
#include <queue>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "su065d4380_interface/commander/common.hpp"

namespace su065d4380_interface
{
class PacketPool
{
public:
  enum class PACKET_TYPE
  {
    VELOCITY,
    INFO,
    PARAM,
    END_PACKET_TYPE
  };

private:
  std::map<PACKET_TYPE, std::queue<std::string>> queue_map_;

public:
  PacketPool();
  ~PacketPool();

  void enqueue(const std::string &);
  bool takePacket(const PACKET_TYPE &, std::string &);

private:
  bool isVelocityPacket(const std::string &) const noexcept;
  bool isInfoPacket(const std::string &) const noexcept;
  bool isParamPacket(const std::string &) const noexcept;

  static const rclcpp::Logger getLogger() noexcept;
  static const std::string fixEscapeSequence(const std::string &);
};
}  // namespace su065d4380_interface
