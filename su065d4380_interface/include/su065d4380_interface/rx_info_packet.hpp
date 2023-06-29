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

namespace su065d4380_interface
{
class RxInfoPacketBase : public h6x_packet_handler::RxPacket<3, 8, 3>
{
public:
  using RxPacket<3, 8, 3>::RxPacket;
};

class RxVelPacketBase : public RxInfoPacketBase
{
public:
  using RxInfoPacketBase::RxInfoPacketBase;

  uint8_t getMode()
  {
    return this->bin_data[0];
  }

  int16_t getRPM()
  {
    return this->get2byteData<int16_t>(2);
  }
};

class RxLeftVelPacket : public RxVelPacketBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxLeftVelPacket)

public:
  RxLeftVelPacket()
  : RxVelPacketBase({'$', 'A', '1'}) {}
};

class RxRightVelPacket : public RxVelPacketBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxRightVelPacket)

public:
  RxRightVelPacket()
  :  RxVelPacketBase({'$', 'A', '2'}) {}
};

class RxDrvPacket : public RxInfoPacketBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxDrvPacket)

public:
  RxDrvPacket()
  : RxInfoPacketBase({'$', 'A', '3'}) {}

  uint16_t getDriverState()
  {
    return this->get2byteData<uint16_t>(0);
  }

  uint16_t getErrorState()
  {
    return this->get2byteData<uint16_t>(2);
  }
};

class RxEncPacket : public RxInfoPacketBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxEncPacket)

private:
  int64_t left_, right_;

public:
  RxEncPacket()
  : RxInfoPacketBase({'$', 'A', '4'})
  {
    this->reset();
    this->makeOK();
  }

  inline void consumed() = delete;

  bool set(const std::string & buf) noexcept override
  {
    if (!this->setBase(buf)) {
      return false;
    }

    this->right_ += this->get2byteData<int16_t>(0);
    this->left_ += this->get2byteData<int16_t>(2);

    return true;
  }

  int64_t getRightEncoder()
  {
    return this->right_;
  }

  int64_t getLeftEncoder()
  {
    return this->left_;
  }

  void reset()
  {
    left_ = 0;
    right_ = 0;
  }
};

class RxVolPacket : public RxInfoPacketBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxVolPacket)

public:
  RxVolPacket()
  : RxInfoPacketBase({'$', 'A', '5'}) {}

  double getVoltage()
  {
    return
      static_cast<double>(this->get2byteData<uint16_t>(0) * 1e-2);
  }
};
}  // namespace su065d4380_interface
