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
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <su065d4380_interface/su065d4380_interface.hpp>
namespace su065d4380_control
{
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using rclcpp_lifecycle::State;
class SU065D4380System : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SU065D4380System)

private:
  using Interface = su065d4380_interface::SU065D4380Interface;
  Interface::SharedPtr interface_;

  double left_reduction_ratio_ = 1.0;
  double right_reduction_ratio_ = 1.0;

  std::vector<double> command_velocities_, state_positions_, state_velocities_;

public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  CallbackReturn on_configure(const State &) override;
  CallbackReturn on_activate(const State &) override;
  CallbackReturn on_deactivate(const State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
private:
  static const rclcpp::Logger getLogger() noexcept;
};
}  // namespace su065d4380_control
