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
class RxInfoPacketBase : public h6x_packet_handler::RxPacket<3, 8, 2>
{
public:
  static const char ID = 'A';
  static const std::size_t ID_IDX = 1;
  static const std::size_t SUB_ID_IDX = 2;

public:
  RxInfoPacketBase() = delete;
  explicit RxInfoPacketBase(const char sub_id)
  : h6x_packet_handler::RxPacket<3, 8, 2>::RxPacket({'$', ID, sub_id}) {}
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
  static const char SUB_ID = '1';

public:
  RxLeftVelPacket()
  : RxVelPacketBase(SUB_ID) {}
};

class RxRightVelPacket : public RxVelPacketBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxRightVelPacket)
  static const char SUB_ID = '2';

public:
  RxRightVelPacket()
  :  RxVelPacketBase(SUB_ID) {}
};

enum class driver_state_t : uint16_t
{
  OK = 0,
  ERROR = 1 << 0,
  LOW_VOLTAGE_WARN = 1 << 1,
  LOW_VOLTAGE = 1 << 2,
  LOW_VOLTAGE_EMERGENCY = 1 << 3,
};

enum class error_state_t : uint16_t
{
  OK = 0,
  LOW_VOLTAGE = 1 << 0,
  HIGH_VOLTAGE = 1 << 1,
  INTERNAL_DRIVER_ERROR = 1 << 2,
  SENSOR_ERROR = 1 << 3,
  OVER_CURRENT = 1 << 4,
  INVALID_VELOCITY = 1 << 5,  // Solvable
  OVER_LOAD = 1 << 6,  // Solvable
  COMMUNICATION_ERROR = 1 << 7  // Solvable
};

class RxDrvPacket : public RxInfoPacketBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxDrvPacket)
  static const char SUB_ID = '3';

public:
  RxDrvPacket()
  : RxInfoPacketBase(SUB_ID) {}

  driver_state_t getDriverState()
  {
    return static_cast<driver_state_t>(this->get2byteData<uint16_t>(0));
  }

  error_state_t getErrorState()
  {
    return static_cast<error_state_t>(this->get2byteData<uint16_t>(2));
  }

  static bool isSolvableError(const error_state_t & e)
  {
    switch (e) {
      case error_state_t::OK:
        // Actually this is not error though
        return true;
      case error_state_t::LOW_VOLTAGE:
        [[fallthrough]];
      case error_state_t::HIGH_VOLTAGE:
        [[fallthrough]];
      case error_state_t::INTERNAL_DRIVER_ERROR:
        [[fallthrough]];
      case error_state_t::SENSOR_ERROR:
        [[fallthrough]];
      case error_state_t::OVER_CURRENT:
        return false;
      case error_state_t::INVALID_VELOCITY:
        [[fallthrough]];
      case error_state_t::OVER_LOAD:
        [[fallthrough]];
      case error_state_t::COMMUNICATION_ERROR:
        return true;
      default:
        break;
    }
    return false;
  }

  static std::string getErrorStr(const error_state_t & e)
  {
    switch (e) {
      case error_state_t::OK:
        return "OK";
      case error_state_t::LOW_VOLTAGE:
        return "Low voltage";
      case error_state_t::HIGH_VOLTAGE:
        return "High voltage";
      case error_state_t::INTERNAL_DRIVER_ERROR:
        return "Internal driver error";
      case error_state_t::SENSOR_ERROR:
        return "Sensor error";
      case error_state_t::OVER_CURRENT:
        return "Over current";
      case error_state_t::INVALID_VELOCITY:
        return "Invalid velocity";
      case error_state_t::OVER_LOAD:
        return "Overload";
      case error_state_t::COMMUNICATION_ERROR:
        return "Communication error";
      default:
        break;
    }
    return "Unknown";
  }
};

class RxEncPacket : public RxInfoPacketBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxEncPacket)
  static const char SUB_ID = '4';

private:
  int64_t left_, right_;

public:
  RxEncPacket()
  : RxInfoPacketBase(SUB_ID)
  {
    this->reset();
    this->makeOK();
  }

  inline void consume() = delete;

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
    this->left_ = 0;
    this->right_ = 0;
  }
};

class RxVolPacket : public RxInfoPacketBase
{
public:
  RCLCPP_UNIQUE_PTR_DEFINITIONS(RxVolPacket)
  static const char SUB_ID = '5';

public:
  RxVolPacket()
  : RxInfoPacketBase(SUB_ID) {}

  double getVoltage()
  {
    return
      static_cast<double>(this->get2byteData<uint16_t>(0) * 1e-2);
  }
};
}  // namespace su065d4380_interface
