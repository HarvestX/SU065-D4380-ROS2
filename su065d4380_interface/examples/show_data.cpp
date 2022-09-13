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


static const rclcpp::Logger getLogger()
{
  return rclcpp::get_logger("ShowData");
}

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  rclcpp::init(argc, argv);
  std::string port_name = "/dev/ttyUSB0";

  RCLCPP_INFO(
    getLogger(),
    "Selected dev: %s", port_name.c_str());


  auto port_handler =
    std::make_shared<su065d4380_interface::PortHandler>(port_name);

  if (!port_handler->openPort()) {
    RCLCPP_ERROR(getLogger(), "Failed to open the port !");
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(
    getLogger(), "Succeeded to open the port !");
  RCLCPP_INFO(
    getLogger(), "BaudRate : %d", port_handler->getBaudRate());

  auto packet_handler =
    std::make_shared<su065d4380_interface::PacketHandler>(port_handler.get());

  auto velocity_commander =
    std::make_unique<su065d4380_interface::VelocityCommander>(
    packet_handler, 500ms);

  auto info_commander =
    std::make_unique<su065d4380_interface::InfoCommander>(
    packet_handler, 500ms);

  velocity_commander->writeVelocity(
    su065d4380_interface::FLAG_MODE_MOTOR_ON, 0, 0);

  rclcpp::Clock::SharedPtr clock =
    std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

  auto time_started = clock->now();
  float voltage = 0.0;
  while (clock->now() - time_started < rclcpp::Duration(5s)) {
    packet_handler->readPortIntoQueue();
    info_commander->evaluateResponse();

    su065d4380_interface::RESPONSE_STATE response_state =
      info_commander->readVoltage(voltage);

    if (response_state != su065d4380_interface::RESPONSE_STATE::OK) {
      su065d4380_interface::CommandUtil::logResponse(
        getLogger(),
        response_state);
    } else {
      RCLCPP_INFO(
        getLogger(),
        "Voltage %.3f [V]",
        voltage);
    }

    rclcpp::sleep_for(100ms);
  }

  return EXIT_SUCCESS;
}
