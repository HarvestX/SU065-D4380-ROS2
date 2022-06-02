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
#include <exception>
#include <stdexcept>
#include "su065d4380_interface/packet/common.hpp"

namespace su065d4380_interface
{
namespace velocity_packet
{
// +1 for null
static constexpr size_t RX_PACKET_SIZE = 6 + 1;
static constexpr size_t TX_PACKET_SIZE = 18 + 1;

const char * const RX_OK = "$8C06\r";
const char * const RX_NG = "$8C07\r";

const uint16_t FLAG_MODE_MOTOR_ON = 0b00000001;
const uint16_t FLAG_MODE_BREAK_OFF = 0b00000100;
const uint16_t FLAG_MODE_ERROR_REST = 0b00010000;

enum class TX_IDX
{
  PREFIX = 0,
  COMMAND_ID = 1,
  DATA1 = 3,
  DATA2 = 5,
  DATA3 = 7,
  DATA4 = 9,
  DATA5 = 11,
  DATA6 = 13,
  CHECK_CODE = 15,
  CR = 17
};

void setPacket(
  const uint16_t,
  const int32_t,
  const int32_t,
  std::string &);
bool isOK(const std::string &);
}  // namespace velocity_packet
}  // namespace su065d4380_interface
