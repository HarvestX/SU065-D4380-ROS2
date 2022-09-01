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

#include "su065d4380_interface/port_handler.hpp"
#include "su065d4380_interface/packet_handler.hpp"
#include "su065d4380_interface/commander/info_commander.hpp"
#include "su065d4380_interface/commander/parameter_commander.hpp"
#include "su065d4380_interface/commander/velocity_commander.hpp"


namespace su065d4380_interface
{
using namespace std::chrono_literals;
class SU065D4380Interface
{
private:
  std::unique_ptr<PortHandler> port_handler_;
  std::shared_ptr<PacketHandler> packet_handler_;

  std::unique_ptr<VelocityCommander> velocity_commander_;
  std::unique_ptr<InfoCommander> info_commander_;

  bool last_velocity_command_accepted;

  rclcpp::Clock::SharedPtr clock_;
  const rclcpp::Duration TIMEOUT_;

public:
  SU065D4380Interface() = delete;
  explicit SU065D4380Interface(
    const std::string &,
    const std::chrono::nanoseconds = 1s);

  bool init();
  bool activate();
  bool deactivate();

  bool readPreprocess()const noexcept;
  bool readLastVelocityCommandState() noexcept;
  bool readRightRpm(int16_t &)const noexcept;
  bool readLeftRpm(int16_t &) const noexcept;
  bool readEncoder(double &, double &) const noexcept;
  bool readError() const noexcept;

  bool writeVelocity(const int16_t &, const int16_t &) noexcept;

private:
  static const rclcpp::Logger getLogger() noexcept;
  static bool processResponse(const RESPONSE_STATE &) noexcept;
};

}  // namespace su065d4380_interface
