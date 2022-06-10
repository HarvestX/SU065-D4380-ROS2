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

#include <hardware_interface/visibility_control.h>

#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <su065d4380_interface/packet_handler.hpp>


namespace su065d4380_control
{
class SU065D4380System : public hardware_interface::SystemInterface
{
private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  std::unique_ptr<su065d4380_interface::PacketHandler> hw_if_;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SU065D4380System)

  HARDWARE_INTERFACE_PUBLIC
  CallbackReturn
  on_init(const hardware_interface::HardwareInfo & info) override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  HARDWARE_INTERFACE_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  static rclcpp::Logger getLogger();
};
}  // namespace su065d4380_control
