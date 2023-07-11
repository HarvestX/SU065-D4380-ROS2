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

#include <h6x_packet_handler/rx_ascii_packet.hpp>
#include <h6x_packet_handler/tx_ascii_packet.hpp>
#include <h6x_packet_handler/tx_rx_packet_base.hpp>

namespace su065d4380_interface
{

enum class mode_flag_t : uint8_t
{
  FLAG_MODE_MOTOR_ON = 1 << 0,
  FLAG_MODE_BREAK_OFF = 1 << 2,
  FLAG_MODE_ERROR_REST = 1 << 4,
};

constexpr mode_flag_t operator|(const mode_flag_t & lhs, const mode_flag_t & rhs)
{
  return static_cast<mode_flag_t>(
    static_cast<uint8_t>(lhs) | static_cast<uint8_t>(rhs));
}

class TxVelPacket : public h6x_packet_handler::TxPacket<3, 12, 2>
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(TxVelPacket)

public:
  TxVelPacket()
  : TxPacket<3, 12, 2>::TxPacket({'$', '8', 'C'})
  {}

  void setVelocity(const mode_flag_t mode, const int16_t right, const int16_t left)
  {
    this->bin_data[0] = static_cast<uint8_t>(mode);

    this->set2ByteData<int16_t>(2, right);
    this->set2ByteData<int16_t>(4, left);

    this->makeOK();
  }
};

class RxVelPacket : public h6x_packet_handler::RxPacket<3, 2, 0>
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxVelPacket)

public:
  RxVelPacket()
  : RxPacket<3, 2, 0>::RxPacket({'$', '8', 'C'})
  {}

  bool isResponseOK()
  {
    if (this->bin_data[0] == 6) {
      return true;
    } else if (this->bin_data[0] == 7) {
      return false;
    }
    throw std::runtime_error("Invalid response");
  }
};

class TxRxVelPacket : public h6x_packet_handler::TxRxPacketBase<TxVelPacket, RxVelPacket>
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(TxRxVelPacket)

public:
  TxRxVelPacket()
  : TxRxPacketBase<TxVelPacket, RxVelPacket>()
  {}

  void setVelocity(const mode_flag_t mode, const int16_t right, const int16_t left)
  {
    this->tx_packet.setVelocity(mode, right, left);
  }

  bool isResponseOK() override
  {
    return this->rx_packet.isResponseOK();
  }
};
}   // namespace su065d4380_interface
