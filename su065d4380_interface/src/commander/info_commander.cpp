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

#include "su065d4380_interface/commander/info_commander.hpp"

namespace su065d4380_interface
{

InfoCommander::InfoCommander(
  std::shared_ptr<PacketHandler> packet_handler,
  const std::chrono::nanoseconds timeout
)
: packet_handler_(packet_handler),
  clock_(std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME)),
  TIMEOUT_(rclcpp::Duration(timeout))
{
}

void InfoCommander::evaluateResponse() const noexcept
{
  std::string response;
  static const int READ_CHECKSUM_IDX = 11;
  while (this->packet_handler_->takeInfoPacket(response)) {
    if (!CommandUtil::confirmChecksum(response, READ_CHECKSUM_IDX)) {
      continue;
    }


  }
}

const rclcpp::Logger InfoCommander::getLogger()
{
  return rclcpp::get_logger("InfoCommander");
}
}  // namespace su065d4380_interface
