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

namespace su065d4380_interface
{
namespace info_packet
{

static constexpr size_t RX_PACKET_SIZE = 14;

enum class RX_IDX
{
  PREFIX = 0,
  COMMAND_ID = 1,
  DATA1 = 3,
  DATA2 = 5,
  DATA3 = 7,
  DATA4 = 9,
  CHECK_CODE = 11,
  CR = 13
};

enum class COMMAND_TYPE
{
  RIGHT_WHEEL,
  LEFT_WHEEL,
  DRIVER_STATE,
  ENCODE_DATA,
  VOLTAGE,
  INVALID
};

enum class VOLTAGE_STATE
{
  FINE,
  LOW_WARNING,
  LOW,
  LOW_EMERGENCY
};

typedef struct
{
  bool working;
  bool has_error;
  VOLTAGE_STATE voltage_state;
} DriverState;

bool checkSum(const std::string &);

COMMAND_TYPE getCommandType(const std::string &);

int32_t getRPM(const std::string &);
void getDriverState(
  const std::string &,
  std::reference_wrapper<std::unique_ptr<DriverState>>);
std::string getErrorState(const std::string &);
uint32_t getRightEncoderData(const std::string &);
uint32_t getLeftEncoderData(const std::string &);
float getVoltage(const std::string &);


}  // namespace info_packet
}  // namespace su065d4380_interface
