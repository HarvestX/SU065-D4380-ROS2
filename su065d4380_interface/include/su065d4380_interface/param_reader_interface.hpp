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

#include "su065d4380_interface/interface_base.hpp"
#include "su065d4380_interface/tx_rx_param_read_packet.hpp"

namespace su065d4380_interface
{
class ParamReaderInterface : public InterfaceBase
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ParamReaderInterface)

private:
  TxRxParamReadPacket::UniquePtr tx_rx_param_read_packet_;

public:
  using InterfaceBase::InterfaceBase;
  ~ParamReaderInterface();

  CallbackReturn on_init() override;

  void readRightGain();
  void readLeftGain();
  void readAccelerationTime();
  void readDecelerationTime();
  void readTimeout();
  void readInputOffDecelerationTime();

protected:
  void readSinglePacket(const std::string &) override;
  static const rclcpp::Logger getLogger() noexcept;
};
}  // namespace su065d4380_interface
