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


#include "su065d4380_interface/port_handler.hpp"
#include "su065d4380_interface/packet_handler.hpp"

int main(int argc, char ** argv)
{
  std::string port_name =
    "/dev/ttyUSB0";
  if (argc == 2) {
    port_name = argv[1];
  }

  const rclcpp::Logger logger = rclcpp::get_logger("ShowData");

  auto port_handler =
    std::make_shared<su065d4380_interface::PortHandler>(port_name);

  if (!port_handler->openPort()) {
    RCLCPP_ERROR(logger, "Failed to open the port!");
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(
    logger, "Succeeded to open the port!");
  RCLCPP_INFO(
    logger, "BaudRate: %d", port_handler->getBaudRate());

  auto packet_handler =
    std::make_unique<su065d4380_interface::PacketHandler>(port_handler);

  packet_handler->sendVelocityCommand(0.0, 0.0);

  while (1) {
    packet_handler->recvCommand();

    RCLCPP_INFO(
      logger,
      "Position left %.3lf, right %.3lf",
      packet_handler->getLeftPosition(),
      packet_handler->getRightPosition());

    RCLCPP_INFO(
      logger,
      "Velocity left %.3lf, right %.3lf",
      packet_handler->getLeftVelocity(),
      packet_handler->getRightVelocity());

    using namespace std::chrono_literals;
    rclcpp::sleep_for(1s);
  }

  return EXIT_SUCCESS;
}
