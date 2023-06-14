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

  using Interface = su065d4380_interface::SU065D4380Interface;
  const auto logger = std::make_shared<su065d4380_interface::LoggingInterface>();

  auto interface = std::make_unique<Interface>(port_name, logger);

  RCLCPP_INFO(logger->get_logger(), "Selected dev: %s", port_name.c_str());


  if (!interface->init()) {
    RCLCPP_ERROR(logger->get_logger(), "Failed to init");
    return EXIT_FAILURE;
  }

  if (!interface->activate()) {
    RCLCPP_ERROR(logger->get_logger(), "Failed to activate");
    return EXIT_FAILURE;
  }


  rclcpp::Clock::SharedPtr clock =
    std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);

  auto time_started = clock->now();
  while (clock->now() - time_started < rclcpp::Duration(5s)) {
    if (!interface->readPreprocess()) {
      continue;
    }

    interface->readError();

    rclcpp::sleep_for(100ms);
  }

  interface->deactivate();
  return EXIT_SUCCESS;
}
