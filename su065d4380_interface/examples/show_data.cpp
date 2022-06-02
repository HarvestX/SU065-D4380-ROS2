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

int main()
{
  const rclcpp::Logger logger = rclcpp::get_logger("ShowData");
  const std::string port_name =
    "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AB0PCR2T-if00-port0";

  auto port_handler =
    std::make_unique<su065d4380_interface::PortHandler>(port_name);

  if (!port_handler->openPort()) {
    RCLCPP_ERROR(logger, "Failed to open the port!");
    return EXIT_FAILURE;
  }
  RCLCPP_INFO(
    logger, "Succeeded to open the port!");
  RCLCPP_INFO(
    logger, "BaudRate: %d", port_handler->getBaudRate());

  namespace vp = su065d4380_interface::velocity_packet;

  std::string velocity_command;
  su065d4380_interface::velocity_packet::setPacket(
    vp::FLAG_MODE_MOTOR_ON | vp::FLAG_MODE_ERROR_REST,
    3000, 0, velocity_command);


  port_handler->writePort(velocity_command);

  while (true) {
    if (port_handler->getBytesAvailable() > 0) {
      break;
    }
  }

  char buf[100];
  port_handler->readPort(buf, sizeof(buf));
  RCLCPP_INFO(logger, "Recv %s", buf);
  if (su065d4380_interface::velocity_packet::isOK(std::string(buf))) {
    RCLCPP_INFO(logger, "OK!");
  } else {
    RCLCPP_INFO(logger, "NG!");
  }

  return EXIT_SUCCESS;
}
