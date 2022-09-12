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

#include "su065d4380_interface/packet_pool.hpp"
#include <iostream>

namespace su065d4380_interface
{
PacketPool::PacketPool()
{
}

PacketPool::~PacketPool()
{
  for (size_t i = 0;
    i < static_cast<size_t>(PACKET_TYPE::END_PACKET_TYPE);
    ++i
  )
  {
    auto queue = this->queue_map_[static_cast<PACKET_TYPE>(i)];
    while (!queue.empty()) {
      queue.pop();
    }
  }
}

void PacketPool::enqueue(const std::string & in_packet)
{
  static std::string previous_chunk = "";
  std::string chunk = previous_chunk + in_packet;

  std::vector<std::string> packet_candidates;
  bool scanning = false;
  std::string item = "";
  for (char ch : chunk) {
    // Ignore until prefix found
    if (ch != SU065D4380_PREFIX && !scanning) {
      continue;
    }
    scanning = true;
    if (ch == SU065D4380_PREFIX) {
      item.clear();
    }

    item += ch;
    if (ch != SU065D4380_SUFFIX) {
      continue;
    }

    if (item.empty()) {
      continue;
    }

    if (this->isVelocityPacket(item)) {
      RCLCPP_DEBUG(
        this->getLogger(),
        "Velocity packet enqueued");
      this->queue_map_[PACKET_TYPE::VELOCITY].push(item);
    } else if (this->isInfoPacket(item)) {
      RCLCPP_DEBUG(
        this->getLogger(),
        "Info packet enqueued");
      this->queue_map_[PACKET_TYPE::INFO].push(item);
    } else if (this->isParamPacket(item)) {
      RCLCPP_DEBUG(
        this->getLogger(),
        "Param packet enqueued");
      this->queue_map_[PACKET_TYPE::PARAM].push(item);
    } else {
      // Invalid item
      // Do noting
      RCLCPP_WARN(
        this->getLogger(),
        "Command like packet [%s] given. Ignored.",
        PacketPool::fixEscapeSequence(item).c_str());
    }
    scanning = false;
    item.clear();
  }
  if (!item.empty()) {
    previous_chunk = item;
  }
}

bool PacketPool::takePacket(const PACKET_TYPE & type, std::string & packet)
{
  bool res = false;
  packet.clear();
  if (!this->queue_map_[type].empty()) {
    packet = this->queue_map_[type].front();
    this->queue_map_[type].pop();
    res = true;
  }
  return res;
}

bool PacketPool::isVelocityPacket(const std::string & packet) const noexcept
{
  return packet.size() == 6 && packet.at(1) == '8';
}

bool PacketPool::isInfoPacket(const std::string & packet) const noexcept
{
  // TODO(m12watanabe1a): write check sum validation here
  return packet.size() == 14 && packet.at(1) == 'A';
}

bool PacketPool::isParamPacket(const std::string & packet) const noexcept
{
  return (packet.size() == 8 || packet.size() == 12) &&
         packet.at(1) == '0';
}

const rclcpp::Logger PacketPool::getLogger() noexcept
{
  return rclcpp::get_logger("PacketPool");
}

const std::string PacketPool::fixEscapeSequence(const std::string & in)
{
  std::string out;
  for (auto c : in) {
    switch (c) {
      case '\r':
        out += "\\r";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\0':
        out += "\\0";
        break;
      default:
        out += c;
    }
  }
  return out;
}

}  // namespace su065d4380_interface
