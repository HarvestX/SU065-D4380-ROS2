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

#include <h6x_serial_interface/h6x_serial_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "su065d4380_interface/rx_info_packet.hpp"
#include "su065d4380_interface/tx_rx_vel_packet.hpp"


namespace su065d4380_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using State = rclcpp_lifecycle::State;
class SU065D4380Interface : public rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SU065D4380Interface)

  static constexpr double RPM2RPS = M_2_PI / 60.0;
  static constexpr double RPS2RPM = 60.0 / M_2_PI;
  static constexpr double ENC2RAD = M_2_PI / (1 << 14);

private:
  using PortHandler = h6x_serial_interface::PortHandler;
  PortHandler::UniquePtr port_handler_;

  RxLeftVelPacket::UniquePtr rx_left_vel_packet_;
  RxRightVelPacket::UniquePtr rx_right_vel_packet_;
  RxDrvPacket::UniquePtr rx_drv_packet_;
  RxEncPacket::UniquePtr rx_enc_packet_;
  RxVolPacket::UniquePtr rx_vol_packet_;
  TxRxVelPacket::UniquePtr tx_rx_vel_packet_;

public:
  SU065D4380Interface() = delete;
  explicit SU065D4380Interface(const std::string &);
  ~SU065D4380Interface();

  CallbackReturn on_configure(const State &) override;
  CallbackReturn on_activate(const State &) override;
  CallbackReturn on_deactivate(const State &) override;

  void read() noexcept;
  void write() noexcept;
  void consumeAll() noexcept;

  bool hasError() noexcept;
  double getRightVelocity() noexcept;
  double getLeftVelocity() noexcept;
  double getRightRadian() noexcept;
  double getLeftRadian() noexcept;
  void setVelocity(const double, const double) noexcept;

private:
  static const rclcpp::Logger getLogger() noexcept;
};

}  // namespace su065d4380_interface
