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


#include "su065d4380_interface/packet/velocity_packet.hpp"

namespace su065d4380_interface
{
namespace velocity_packet
{
void setPacket(
  const uint16_t mode,
  const int32_t right_rpm,
  const int32_t left_rpm,
  std::string & packet
)
{
  // add 1 for null sequence
  packet.resize(TX_PACKET_SIZE + 1);

  char buf[TX_PACKET_SIZE];
  snprintf(
    buf, sizeof(buf),
    "$8C%02hhX00%04hX%04hX", mode, right_rpm, left_rpm);

  // Calculate checksum
  uint16_t sum = 0;
  for (size_t i = 0; i < static_cast<size_t>(TX_IDX::CHECK_CODE); ++i) {
    sum ^= static_cast<uint16_t>(buf[i]);
  }

  // Set Checksum
  snprintf(
    packet.data(), packet.size(), "%s%02hhX\r", buf, sum);
  packet.erase(std::find(packet.begin(), packet.end(), '\0'), packet.end());
}

bool isOK(const std::string & recv)
{
  if (recv == RX_OK) {
    return true;
  } else if (recv == RX_NG) {
    return false;
  }
  throw std::runtime_error("Invalid response: " + recv);
}
}   // namespace velocity_packet

}  // namespace su065d4380_interface
