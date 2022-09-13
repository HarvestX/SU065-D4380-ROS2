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

#include "su065d4380_interface/packet_pool.hpp"
#include "su065d4380_interface/port_handler_base.hpp"

namespace su065d4380_interface
{

class PacketHandler
{
private:
  PortHandlerBase const * const port_handler_;
  std::unique_ptr<PacketPool> pool_;

public:
  PacketHandler() = delete;
  explicit PacketHandler(
    PortHandlerBase const * const);

  size_t writePort(char const * const, const size_t) const;
  size_t readPortIntoQueue();
  size_t getBytesAvailable() const;

  bool takePacket(const PacketPool::PACKET_TYPE &, std::string &);
};
}  // namespace su065d4380_interface
