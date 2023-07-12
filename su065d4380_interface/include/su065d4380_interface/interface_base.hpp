// Copyright 2023 HarvestX Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <h6x_serial_interface/h6x_serial_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>

namespace su065d4380_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;

class InterfaceBase : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  static constexpr double RPM2RPS = 2.0 * M_PI / 60.0;
  static constexpr double RPS2RPM = 60.0 / 2.0 / M_PI;
  static constexpr double ENC2RAD = 2.0 * M_PI / (1 << 14);

protected:
  using PortHandler = h6x_serial_interface::PortHandler;
  PortHandler::UniquePtr port_handler_;

public:
  InterfaceBase() = delete;
  explicit InterfaceBase(const std::string &);
  virtual ~InterfaceBase();

  void read();

  virtual CallbackReturn on_init() = 0;
  virtual CallbackReturn on_configure(const State &);
  virtual CallbackReturn on_activate(const State &);
  virtual CallbackReturn on_deactivate(const State &);

protected:
  virtual void readSinglePacket(const std::string &) = 0;
};
}  // namespace su065d4380_interface
