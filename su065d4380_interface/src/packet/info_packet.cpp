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

#include "su065d4380_interface/packet/info_packet.hpp"


namespace su065d4380_interface
{
namespace info_packet
{
bool checkSum(const std::string & packet)
{
  const size_t check_code_idx =
    static_cast<size_t>(RX_IDX::CHECK_CODE);

  uint16_t expected;
  try {
    expected =
      std::stoi(packet.substr(check_code_idx), nullptr, 16);
  } catch (std::invalid_argument &) {
    return false;
  }

  uint16_t actual = 0;
  for (size_t i = 0; i < check_code_idx; ++i) {
    actual ^= static_cast<uint16_t>(packet.at(i));
  }

  return expected == actual;
}

COMMAND_TYPE getCommandType(const std::string & packet)
{
  const size_t COMMAND_IDX = static_cast<size_t>(RX_IDX::COMMAND_ID);
  const std::string command_id =
    packet.substr(COMMAND_IDX, 2);

  if (command_id == "A1") {
    return COMMAND_TYPE::RIGHT_WHEEL;
  } else if (command_id == "A2") {
    return COMMAND_TYPE::LEFT_WHEEL;
  } else if (command_id == "A3") {
    return COMMAND_TYPE::DRIVER_STATE;
  } else if (command_id == "A4") {
    return COMMAND_TYPE::ENCODE_DATA;
  } else if (command_id == "A5") {
    return COMMAND_TYPE::VOLTAGE;
  }

  return COMMAND_TYPE::INVALID;
}

int32_t getRPM(const std::string & packet)
{
  const size_t DATA_IDX = static_cast<size_t>(RX_IDX::DATA3);
  return std::stoi(packet.substr(DATA_IDX, 4), nullptr, 16);
}

void getDriverState(
  const std::string & packet,
  std::reference_wrapper<std::unique_ptr<DriverState>> result)
{
  const size_t DATA_IDX = static_cast<size_t>(RX_IDX::DATA1);
  uint16_t data = static_cast<uint16_t>(
    std::stoi(packet.substr(DATA_IDX, 4), nullptr, 16));

  result.get()->working = (data & (1 << 0));
  result.get()->has_error = (data & (1 << 1));
  if (data & (1 << 2)) {
    result.get()->voltage_state = VOLTAGE_STATE::LOW_WARNING;
  } else if (data & (1 << 3)) {
    result.get()->voltage_state = VOLTAGE_STATE::LOW;
  } else if (data & (1 << 4)) {
    result.get()->voltage_state = VOLTAGE_STATE::LOW_EMERGENCY;
  } else {
    result.get()->voltage_state = VOLTAGE_STATE::FINE;
  }
}

std::string getErrorState(const std::string & packet)
{
  const size_t DATA_IDX = static_cast<size_t>(RX_IDX::DATA3);
  uint16_t data = static_cast<uint16_t>(
    std::stoi(packet.substr(DATA_IDX, 4), nullptr, 16));

  std::stringstream ss;

  if (data & (1 << 0)) {
    ss << "Input voltage too low.";
    ss << std::endl;
  }
  if (data & (1 << 1)) {
    ss << "Input voltage too high.";
    ss << std::endl;
  }
  if (data & (1 << 2)) {
    ss << "Driver Internal error.";
    ss << std::endl;
  }
  if (data & (1 << 3)) {
    ss << "Sensor error.";
    ss << std::endl;
  }
  if (data & (1 << 4)) {
    ss << "Overcurrent.";
    ss << std::endl;
  }
  if (data & (1 << 5)) {
    ss << "Invalid velocity error.";
    ss << std::endl;
  }
  if (data & (1 << 6)) {
    ss << "Overload.";
    ss << std::endl;
  }
  if (data & (1 << 7)) {
    ss << "Communication error.";
    ss << std::endl;
  }
  return ss.str();
}

uint32_t getRightEncoderData(const std::string & packet)
{
  const size_t DATA_IDX = static_cast<size_t>(RX_IDX::DATA1);
  return static_cast<uint32_t>(
    std::stoi(packet.substr(DATA_IDX, 4), nullptr, 16));
}

uint32_t getLeftEncoderData(const std::string & packet)
{
  const size_t DATA_IDX = static_cast<size_t>(RX_IDX::DATA3);
  return static_cast<uint32_t>(
    std::stoi(packet.substr(DATA_IDX, 4), nullptr, 16));
}

float getVoltage(const std::string & packet)
{
  const size_t DATA_IDX = static_cast<size_t>(RX_IDX::DATA1);
  return static_cast<float>(
    std::stoi(
      packet.substr(DATA_IDX, 4),
      nullptr, 16)) * 1e-2;
}

}  // namespace info_packet
}  // namespace su065d4380_interface
