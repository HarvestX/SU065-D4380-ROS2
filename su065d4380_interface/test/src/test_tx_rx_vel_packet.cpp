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

#include <gmock/gmock.h>

#include <su065d4380_interface/tx_rx_vel_packet.hpp>

using namespace su065d4380_interface;  // NOLINT

TEST(TxVelPacket, WriteVelPacket) {
  auto vel_packet = std::make_shared<TxVelPacket>();

  ASSERT_FALSE(vel_packet->isOK());
  vel_packet->setVelocity(mode_flag_t::FLAG_MODE_MOTOR_ON, 3000, 0);

  ASSERT_TRUE(vel_packet->isOK());
  std::string out;
  ASSERT_TRUE(vel_packet->get(out));
  ASSERT_EQ(out, "$8C01000BB8000056\r");
}


TEST(RxVelPacket, ReadResponse) {
  auto vel_packet = std::make_shared<RxVelPacket>();

  ASSERT_FALSE(vel_packet->isOK());
  const std::string dataNG = "$8C07";
  ASSERT_TRUE(vel_packet->set(dataNG));
  ASSERT_TRUE(vel_packet->isOK());
  ASSERT_FALSE(vel_packet->isResponseOK());
  vel_packet->consume();


  ASSERT_FALSE(vel_packet->isOK());
  const std::string dataOK = "$8C06";
  ASSERT_TRUE(vel_packet->set(dataOK));
  ASSERT_TRUE(vel_packet->isOK());
  ASSERT_TRUE(vel_packet->isResponseOK());
  vel_packet->consume();
}


TEST(TxRxVelPacket, WriteAndWaitResponse) {
  std::string tx_ret;
  auto vel_packet = std::make_shared<TxRxVelPacket>();
  // Packet sendable
  ASSERT_TRUE(vel_packet->isOK());

  vel_packet->setVelocity(mode_flag_t::FLAG_MODE_MOTOR_ON, 3000, 0);
  ASSERT_TRUE(vel_packet->getTx(tx_ret));
  ASSERT_EQ(tx_ret, "$8C01000BB8000056\r");
  ASSERT_TRUE(vel_packet->isWaitingResponse());

  // Can not set new TX until response processed
  ASSERT_FALSE(vel_packet->getTx(tx_ret));

  const std::string dataOK = "$8C06";
  ASSERT_TRUE(vel_packet->setRx(dataOK));
  ASSERT_FALSE(vel_packet->isWaitingResponse());
  ASSERT_TRUE(vel_packet->isOK());

  vel_packet->setVelocity(mode_flag_t::FLAG_MODE_MOTOR_ON, 3000, 0);
  ASSERT_TRUE(vel_packet->getTx(tx_ret));
  ASSERT_EQ(tx_ret, "$8C01000BB8000056\r");
  ASSERT_TRUE(vel_packet->isWaitingResponse());

  const std::string dataNG = "$8C07";
  ASSERT_FALSE(vel_packet->setRx(dataNG));
  ASSERT_FALSE(vel_packet->isWaitingResponse());
  ASSERT_TRUE(vel_packet->isOK());
}
