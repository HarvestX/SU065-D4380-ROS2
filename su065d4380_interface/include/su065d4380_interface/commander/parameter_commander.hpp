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
#include <sstream>

#include <rclcpp/rclcpp.hpp>

#include "su065d4380_interface/packet_pool.hpp"
#include "su065d4380_interface/port_handler_base.hpp"


namespace su065d4380_interface
{

namespace parameter_packet
{
namespace ids
{
const char * const WRITE = "00W";
const char * const READ = "00R";
}  // namespace ids

const char * const WRITE_OK = "$00W17*\r";
const char * const WRITE_NG = "$00W17/\r";

const char * const READ_OK = "$00R12DDDDFF*\r";
const char * const READ_NG = "$00R12/\r";

enum class TX_IDX
{
  PREFIX = 0,
  RW = 3,
  COMMAND_ID = 4,
  DATA = 8,
  CHECK_CODE = 12,
  EXIT_CODE = 14,
  CR = 15
};

}  // namespace parameter_packet


using namespace std::chrono_literals;
class ParameterCommander
{
private:
  static const size_t BUF_SIZE_ = 100;
  static const uint MAX_GAIN_ = 100;
  static const uint MAX_ACCTIME_ = 500;
  static const uint MAX_TIMEOUT_ = 5;

  // Use raw pointer for gmock
  const PortHandlerBase * const port_handler_;
  rclcpp::Clock::SharedPtr clock_;
  const rclcpp::Duration TIMEOUT;
  std::shared_ptr<PacketPool> pool_;

public:
  ParameterCommander() = delete;
  explicit ParameterCommander(
    PortHandlerBase *,
    const std::chrono::nanoseconds = 100ms);

  // Write parameter
  bool writeRightWheelGain(const uint);
  bool writeLeftWheelGain(const uint);
  bool writeAccTime(const uint);
  bool writeDecTime(const uint);
  bool writeTimeout(const uint);
  bool writeDecWithTimeout(const uint);

  // Read parameter
  int readRightWheelGain(const uint) const;
  int readLeftWheelGain(const uint) const;
  int readAccTime(const uint) const;
  int readDecTime(const uint) const;
  int readTimeout(const uint) const;
  int readDecWithTimeout(const uint) const;

private:
  static const rclcpp::Logger getLogger();
  int setChecksum(char * const, const int);
  bool evaluateWriteResponse();
};
}  // namespace su065d4380_interface
