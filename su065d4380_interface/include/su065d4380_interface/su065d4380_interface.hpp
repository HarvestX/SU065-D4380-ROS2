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

#include <string>

#include "su065d4380_interface/interface_base.hpp"
#include "su065d4380_interface/rx_info_packet.hpp"
#include "su065d4380_interface/tx_rx_vel_packet.hpp"

namespace su065d4380_interface
{
class SU065D4380Interface : public InterfaceBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SU065D4380Interface)

private:
  RxLeftVelPacket::UniquePtr rx_left_vel_packet_;
  RxRightVelPacket::UniquePtr rx_right_vel_packet_;
  RxDrvPacket::UniquePtr rx_drv_packet_;
  RxEncPacket::UniquePtr rx_enc_packet_;
  RxVolPacket::UniquePtr rx_vol_packet_;
  TxRxVelPacket::UniquePtr tx_rx_vel_packet_;

public:
  using InterfaceBase::InterfaceBase;
  ~SU065D4380Interface();

  CallbackReturn on_init() override;
  CallbackReturn on_activate(const State &) override;

  void write() noexcept;
  void consumeAll() noexcept;

  bool hasError() noexcept;
  void showError() noexcept;
  bool isSolvableError() noexcept;
  void solveError() noexcept;

  double getRightVelocity() noexcept;
  double getLeftVelocity() noexcept;
  double getRightRadian() noexcept;
  double getLeftRadian() noexcept;
  void setVelocity(const double, const double) noexcept;

protected:
  void readSinglePacket(const std::string &) override;

private:
  static const rclcpp::Logger getLogger() noexcept;
};
}  // namespace su065d4380_interface
