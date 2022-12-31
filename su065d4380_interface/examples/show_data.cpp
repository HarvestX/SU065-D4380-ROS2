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

#include "su065d4380_interface/su065d4380_interface.hpp"

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;  // NOLINT
  rclcpp::init(argc, argv);
  std::string port_name = "/dev/ttyUSB0";

  const auto logger = std::make_shared<su065d4380_interface::LoggingInterface>();

  RCLCPP_INFO(logger->get_logger(), "Selected dev: %s", port_name.c_str());


  using PortHandler = h6x_serial_interface::PortHandler;
  auto port_handler = std::make_shared<PortHandler>(port_name, logger);

  if (!port_handler->openPort()) {
    RCLCPP_ERROR(logger->get_logger(), "Failed to open the port !");
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(logger->get_logger(), "Succeeded to open the port !");
  RCLCPP_INFO(logger->get_logger(), "Baudrate: %d", port_handler->BAUDRATE);

  auto packet_handler =
    std::make_shared<su065d4380_interface::PacketHandler>(port_handler.get());

  auto velocity_commander =
    std::make_unique<su065d4380_interface::VelocityCommander>(packet_handler, 500ms);

  auto info_commander =
    std::make_unique<su065d4380_interface::InfoCommander>(packet_handler, 500ms);

  velocity_commander->writeVelocity(
    su065d4380_interface::FLAG_MODE_MOTOR_ON, 0, 0);

  rclcpp::Clock::SharedPtr clock =
    std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

  auto time_started = clock->now();
  float voltage = 0.0;
  while (clock->now() - time_started < rclcpp::Duration(5s)) {
    if (packet_handler->readPortIntoQueue() == -1) {
      RCLCPP_ERROR(logger->get_logger(), "Failed to read port");
      rclcpp::sleep_for(100ms);
      continue;
    }
    info_commander->evaluateResponse();

    su065d4380_interface::RESPONSE_STATE response_state =
      info_commander->readVoltage(voltage);

    if (response_state != su065d4380_interface::RESPONSE_STATE::OK) {
      su065d4380_interface::CommandUtil::logResponse(logger->get_logger(), response_state);
    } else {
      RCLCPP_INFO(logger->get_logger(), "Voltage %.3f [V]", voltage);
    }

    rclcpp::sleep_for(100ms);
  }

  return EXIT_SUCCESS;
}
