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
  auto packet = std::make_shared<TxVelPacket>();

  ASSERT_FALSE(packet->isOK());
  packet->setVelocity(mode_flag_t::FLAG_MODE_MOTOR_ON, 3000, 0);

  ASSERT_TRUE(packet->isOK());
  std::string out;
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$8C01000BB8000056\r");
}


TEST(RxVelPacket, ReadResponse) {
  auto packet = std::make_shared<RxVelPacket>();

  ASSERT_FALSE(packet->isOK());
  const std::string data_ng = "$8C07";
  ASSERT_TRUE(packet->set(data_ng));
  ASSERT_TRUE(packet->isOK());
  ASSERT_FALSE(packet->isResponseOK());
  packet->consume();


  ASSERT_FALSE(packet->isOK());
  const std::string data_ok = "$8C06";
  ASSERT_TRUE(packet->set(data_ok));
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->isResponseOK());
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  const std::string data_inval = "$8C08";
  ASSERT_TRUE(packet->set(data_inval));
  ASSERT_TRUE(packet->isOK());
  ASSERT_THROW(packet->isResponseOK(), std::runtime_error);
}


TEST(TxRxVelPacket, WriteAndWaitResponse) {
  std::string tx_ret;
  auto packet = std::make_shared<TxRxVelPacket>();
  // Packet sendable
  ASSERT_TRUE(packet->isOK());

  packet->setVelocity(mode_flag_t::FLAG_MODE_MOTOR_ON, 3000, 0);
  ASSERT_TRUE(packet->getTx(tx_ret));
  ASSERT_EQ(tx_ret, "$8C01000BB8000056\r");
  ASSERT_TRUE(packet->isWaitingResponse());

  // Can not set new TX until response processed
  ASSERT_FALSE(packet->isOK());
  ASSERT_FALSE(packet->getTx(tx_ret));

  const std::string data_ok = "$8C06";
  ASSERT_TRUE(packet->setRx(data_ok));
  ASSERT_FALSE(packet->isWaitingResponse());
  ASSERT_TRUE(packet->isOK());

  packet->setVelocity(mode_flag_t::FLAG_MODE_MOTOR_ON, 3000, 0);
  ASSERT_TRUE(packet->getTx(tx_ret));
  ASSERT_EQ(tx_ret, "$8C01000BB8000056\r");
  ASSERT_TRUE(packet->isWaitingResponse());

  const std::string data_ng = "$8C07";
  ASSERT_FALSE(packet->setRx(data_ng));
  ASSERT_FALSE(packet->isWaitingResponse());
  ASSERT_TRUE(packet->isOK());
}
