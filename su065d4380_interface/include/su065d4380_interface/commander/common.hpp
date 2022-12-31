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
#include <rclcpp/rclcpp.hpp>

#define SU065D4380_PREFIX '$'
#define SU065D4380_SUFFIX '\r'

namespace su065d4380_interface
{

enum class RESPONSE_STATE
{
  OK,
  WAITING_RESPONSE,
  ERROR_EXPLICIT_NG,
  ERROR_NOT_COMING_YET,
  ERROR_INVALID_INPUT,
  ERROR_CRC,
  ERROR_UNKNOWN,
  ERROR_SENDING,
};

enum class VOLTAGE_STATE
{
  OK,
  LOW_WARNING,
  LOW,
  LOW_EMERGENCY,
  INVALID,
};

enum class ERROR_STATE
{
  LOW_VOLTAGE,
  HIGH_VOLTAGE,
  INTERNAL_DRIVER_ERROR,
  SENSOR_ERROR,
  OVER_CURRENT,
  INVALID_VELOCITY,
  OVER_LOAD,
  COMMUNICATION_ERROR,
  OK,
  INVALID,
};

static const uint16_t FLAG_MODE_MOTOR_ON = 0b00000001;
static const uint16_t FLAG_MODE_BREAK_OFF = 0b00000100;
static const uint16_t FLAG_MODE_ERROR_REST = 0b00010000;

class CommandUtil
{
public:
  static uint16_t calcChecksum(char const * const, const int);
  static int setChecksum(char * const, const size_t, const int);
  static bool confirmChecksum(const std::string &, const int);
  static void logResponse(const rclcpp::Logger &, const RESPONSE_STATE &);
  static void logError(const rclcpp::Logger &, const ERROR_STATE &);
};
}  // namespace su065d4380_interface
