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
class InfoCommander
{
private:
  std::shared_ptr<PacketHandler> packet_handler_;
  rclcpp::Clock::SharedPtr clock_;
  const rclcpp::Duration TIMEOUT_;

public:
  InfoCommander() = delete;
  explicit InfoCommander(
    std::shared_ptr<PacketHandler>,
    const std::chrono::nanoseconds = 1s);

  RESPONSE_STATE read(int &);


  void evaluateResponse() const noexcept;

private:
  static const rclcpp::Logger getLogger();
};
}  // namespace su065d4380_interface
