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

class TxParamReadPacket : public h6x_packet_handler::TxPacket<4, 4, 2>::TxPacket
{
public:
  TxParamReadPacket()
  : TxPacket<4, 4, 2>({'$', '0', '0', 'R'}, "*\r") {}

  void setRightGain()
  {
    this->set2ByteData<uint16_t>(0, 30);
    this->makeOK();
  }

  void setLeftGain()
  {
    this->set2ByteData<uint16_t>(0, 31);
    this->makeOK();
  }

  void setAccelerationTime()
  {
    this->set2ByteData<uint16_t>(0, 32);
    this->makeOK();
  }

  void setDecelerationTime()
  {
    this->set2ByteData<uint16_t>(0, 33);
    this->makeOK();
  }

  void setTimeout()
  {
    this->set2ByteData<uint16_t>(0, 37);
    this->makeOK();
  }

  void setInputOffDecelerationTime()
  {
    this->set2ByteData<uint16_t>(0, 38);
    this->makeOK();
  }
};


class RxParamReadPacket : public h6x_packet_handler::RxPacket<4, 4, 3>::RxPacket
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxParamReadPacket)

protected:
  enum class RESPONSE
  {
    OK,
    NG,
    UNKNOWN
  } response_;

public:
  RxParamReadPacket()
  : RxPacket<4, 4, 3>({'$', '0', '0', 'R'})
  {
    this->response_ = RESPONSE::UNKNOWN;
  }

  bool set(const std::string & buf) noexcept override
  {
    this->response_ = RESPONSE::UNKNOWN;
    if (!this->checkPrefix(buf)) {
      return false;
    }

    if (buf == "$00R12/") {
      this->response_ = RESPONSE::NG;
    } else {
      if (buf.size() != ASCII_BUF_SIZE || !this->checkCRC(buf) || !this->convert(buf)) {
        return false;
      }
      this->response_ = RESPONSE::OK;
    }

    this->makeOK();
    return true;
  }

  bool isResponseOK()
  {
    switch (this->response_) {
      case RESPONSE::OK:
        return true;
      case RESPONSE::NG:
        return false;
      case RESPONSE::UNKNOWN:
        [[fallthrough]];
      default:
        throw std::runtime_error("Unkown Response given");
    }
  }

  uint16_t getData()
  {
    return this->get2byteData<uint16_t>(0);
  }

protected:
  inline bool checkCRC(const std::string & buf) const noexcept override
  {
    return buf[ASCII_BUF_SIZE - 1] == '*' && this->checkCrc8(buf);
  }
};

class TxRxParamReadPacket : public h6x_packet_handler::TxRxPacketBase<TxParamReadPacket,
    RxParamReadPacket>
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(TxRxParamReadPacket)

public:
  TxRxParamReadPacket()
  : TxRxPacketBase<TxParamReadPacket, RxParamReadPacket>()
  {}

  void setRightGain()
  {
    this->tx_packet.setRightGain();
    this->makeOK();
  }

  void setLeftGain()
  {
    this->tx_packet.setLeftGain();
    this->makeOK();
  }

  void setAccelerationTime()
  {
    this->tx_packet.setAccelerationTime();
    this->makeOK();
  }

  void setDecelerationTime()
  {
    this->tx_packet.setDecelerationTime();
    this->makeOK();
  }

  void setTimeout()
  {
    this->tx_packet.setTimeout();
    this->makeOK();
  }

  void setInputOffDecelerationTime()
  {
    this->tx_packet.setInputOffDecelerationTime();
    this->makeOK();
  }

  uint16_t getData() noexcept
  {
    return this->rx_packet.getData();
  }

  bool isResponseOK() override
  {
    return this->rx_packet.isResponseOK();
  }
};
}  // namespace su065d4380_interface
