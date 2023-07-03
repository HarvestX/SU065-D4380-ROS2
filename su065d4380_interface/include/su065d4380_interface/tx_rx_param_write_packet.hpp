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

#include <h6x_packet_handler/rx_ascii_packet.hpp>
#include <h6x_packet_handler/tx_ascii_packet.hpp>
#include <h6x_packet_handler/tx_rx_packet_base.hpp>


namespace su065d4380_interface
{
class TxParamWritePacket : public h6x_packet_handler::TxPacket<4, 8, 2>::TxPacket
{
public:
  TxParamWritePacket()
  : TxPacket<4, 8, 2>({'$', '0', '0', 'W'}, "*\r") {}

  void setRightGain(const uint16_t val)
  {
    this->set2ByteData<uint16_t>(0, 30);
    this->set2ByteData<uint16_t>(2, val);
    this->makeOK();
  }

  void setLeftGain(const uint16_t val)
  {
    this->set2ByteData<uint16_t>(0, 31);
    this->set2ByteData<uint16_t>(2, val);
    this->makeOK();
  }

  void setAccelerationTime(const uint16_t val)
  {
    this->set2ByteData<uint16_t>(0, 32);
    this->set2ByteData<uint16_t>(2, val);
    this->makeOK();
  }

  void setDecelerationTime(const uint16_t val)
  {
    this->set2ByteData<uint16_t>(0, 33);
    this->set2ByteData<uint16_t>(2, val);
    this->makeOK();
  }

  void setTimeout(const uint16_t val)
  {
    this->set2ByteData<uint16_t>(0, 37);
    this->set2ByteData<uint16_t>(2, val);
    this->makeOK();
  }

  void setInputOffDecelerationTime(const uint16_t val)
  {
    this->set2ByteData<uint16_t>(0, 38);
    this->set2ByteData<uint16_t>(2, val);
    this->makeOK();
  }
};

class RxParamWritePacket : public h6x_packet_handler::RxPacket<6, 1, 0>::RxPacket
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxParamWritePacket)

public:
  RxParamWritePacket()
  : RxPacket<6, 1, 0>({'$', '0', '0', 'W', '1', '7'})
  {}

  bool isResponseOK()
  {
    switch (this->bin_data[0]) {
      case '*':
        return true;
      case '/':
        return false;
      default:
        throw std::runtime_error("Invalid response");
    }
  }

protected:
  bool convert(const std::string & buf) noexcept override
  {
    this->bin_data[0] = buf[ASCII_STX_SIZE];
    return true;
  }
};

class TxRxParamWritePacket : public h6x_packet_handler::TxRxPacketBase<TxParamWritePacket,
    RxParamWritePacket>
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(TxRxParamWritePacket)

public:
  TxRxParamWritePacket()
  : TxRxPacketBase<TxParamWritePacket, RxParamWritePacket>()
  {}

  void setRightGain(const uint16_t val)
  {
    this->tx_packet.setRightGain(val);
  }

  void setLeftGain(const uint16_t val)
  {
    this->tx_packet.setLeftGain(val);
  }

  void setAccelerationTime(const uint16_t val)
  {
    this->tx_packet.setAccelerationTime(val);
  }

  void setDecelerationTime(const uint16_t val)
  {
    this->tx_packet.setDecelerationTime(val);
  }

  void setTimeout(const uint16_t val)
  {
    this->tx_packet.setTimeout(val);
  }

  void setInputOffDecelerationTime(const uint16_t val)
  {
    this->tx_packet.setInputOffDecelerationTime(val);
  }

  bool isResponseOK() override
  {
    return this->rx_packet.isResponseOK();
  }
};
}  // namespace su065d4380_interface
