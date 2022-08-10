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

#include <queue>
#include <memory>
#include <string>

#include "su065d4380_interface/commander/common.hpp"

namespace su065d4380_interface
{
class PacketPool
{
private:
  std::string previous_chunk_;

  std::unique_ptr<std::queue<std::string>> velocity_queue_;
  std::unique_ptr<std::queue<std::string>> info_queue_;
  std::unique_ptr<std::queue<std::string>> param_queue_;

public:
  PacketPool();
  ~PacketPool();

  void enqueue(const std::string &);

  bool takeVelocityPacket(std::string &);
  bool takeInfoPacket(std::string &);
  bool takeParamPacket(std::string &);

private:
  bool isVelocityPacket(const std::string &) const noexcept;
  bool isInfoPacket(const std::string &) const noexcept;
  bool isParamPacket(const std::string &) const noexcept;
};
}  // namespace su065d4380_interface
