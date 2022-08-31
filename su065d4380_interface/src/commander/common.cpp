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


#include "su065d4380_interface/commander/common.hpp"

namespace su065d4380_interface
{
uint16_t CommandUtil::calcChecksum(
  char const * const buf, const int cx)
{
  uint16_t crc = 0;
  for (int i = 0; i < cx; ++i) {
    crc ^= static_cast<uint16_t>(buf[i]);
  }
  return crc;
}

int CommandUtil::setChecksum(
  char * const buf, const size_t buf_size, const int cx)
{
  const uint16_t crc = CommandUtil::calcChecksum(buf, cx);
  return snprintf(buf + cx, buf_size - cx, "%02hhX\r", crc) + cx;
}

bool CommandUtil::confirmChecksum(const std::string & buf, const int CRC_IDX)
{

  uint16_t expected_crc;
  try {
    expected_crc =
      std::stoi(buf.substr(CRC_IDX, 2), nullptr, 16);
  } catch (std::invalid_argument &) {
    return false;
  }

  const uint16_t actual_crc = CommandUtil::calcChecksum(
    buf.data(), CRC_IDX);

  if (expected_crc != actual_crc) {
    RCLCPP_ERROR(
      CommandUtil::getLogger(),
      "Expected crc: [%02hhX] Actual crc: [%02hhX]",
      expected_crc, actual_crc);
  }

  return expected_crc == actual_crc;
}

void CommandUtil::logResponse(
  const rclcpp::Logger & logger,
  const RESPONSE_STATE & response)
{
  switch (response) {
    case RESPONSE_STATE::OK:
      RCLCPP_INFO(
        logger,
        "Response OK");
      break;
    case RESPONSE_STATE::WAITING_RESPONSE:
      RCLCPP_INFO(
        logger,
        "Waiting for response");
      break;
    case RESPONSE_STATE::ERROR_EXPLICIT_NG:
      RCLCPP_ERROR(
        logger,
        "Driver response NG");
      break;
    case RESPONSE_STATE::ERROR_NOT_COMING_YET:
      RCLCPP_ERROR(
        logger,
        "Driver not responded yet");
      break;
    case RESPONSE_STATE::ERROR_INVALID_INPUT:
      RCLCPP_ERROR(
        logger,
        "Invalid input given");
      break;
    case RESPONSE_STATE::ERROR_CRC:
      RCLCPP_ERROR(
        logger,
        "Crc calculation failed");
      break;
    case RESPONSE_STATE::ERROR_UNKNOWN:
      RCLCPP_ERROR(
        logger,
        "Uknown error");
      break;
  }
}

void CommandUtil::logError(
  const rclcpp::Logger & logger,
  const ERROR_STATE & error)
{
  switch (error) {
    case ERROR_STATE::OK:
      RCLCPP_INFO(
        logger,
        "OK");
      break;
    case ERROR_STATE::HIGH_VOLTAGE:
      RCLCPP_ERROR(
        logger,
        "High Voltage");
      break;
    case ERROR_STATE::INTERNAL_DRIVER_ERROR:
      RCLCPP_ERROR(
        logger,
        "Internal Driver Error");
      break;
    case ERROR_STATE::SENSOR_ERROR:
      RCLCPP_ERROR(
        logger,
        "Sensor Error");
      break;
    case ERROR_STATE::OVER_CURRENT:
      RCLCPP_ERROR(
        logger,
        "Over Current");
      break;
    case ERROR_STATE::INVALID_VELOCITY:
      RCLCPP_ERROR(
        logger,
        "Invalid Velocity");
      break;
    case ERROR_STATE::OVER_LOAD:
      RCLCPP_ERROR(
        logger,
        "Over Load");
      break;
    case ERROR_STATE::COMMUNICATION_ERROR:
      RCLCPP_ERROR(
        logger,
        "Communication Error");
      break;
    case ERROR_STATE::INVALID:
    default:
      break;
  }
}

const rclcpp::Logger CommandUtil::getLogger()
{
  return rclcpp::get_logger("CommandUtil");
}
}  // namespace su065d4380_interface
