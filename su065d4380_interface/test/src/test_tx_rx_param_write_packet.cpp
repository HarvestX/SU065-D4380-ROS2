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

#include <su065d4380_interface/tx_rx_param_write_packet.hpp>

using namespace su065d4380_interface;  // NOLINT

TEST(TxRxParamWritePacket, WriteData) {
  std::string out;
  auto packet = std::make_unique<TxParamWritePacket>();

  ASSERT_FALSE(packet->isOK());
  packet->setRightGain(100);
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00W001E006405*\r");
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  packet->setLeftGain(100);
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00W001F006406*\r");
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  packet->setAccelerationTime(500);
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00W002001F402*\r");
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  packet->setDecelerationTime(500);
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00W002101F403*\r");
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  packet->setTimeout(5);
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00W0025000571*\r");
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  packet->setInputOffDecelerationTime(500);
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00W002601F404*\r");
  packet->consume();
}

TEST(TxRxParamWritePacket, ReadResponse) {
  auto packet = std::make_unique<RxParamWritePacket>();

  ASSERT_FALSE(packet->isOK());
  const std::string data_ng = "$00W17/";
  ASSERT_TRUE(packet->set(data_ng));
  ASSERT_TRUE(packet->isOK());
  ASSERT_FALSE(packet->isResponseOK());
  packet->consume();


  ASSERT_FALSE(packet->isOK());
  const std::string data_ok = "$00W17*";
  ASSERT_TRUE(packet->set(data_ok));
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->isResponseOK());
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  const std::string data_inval = "$00W17#";
  ASSERT_TRUE(packet->set(data_inval));
  ASSERT_TRUE(packet->isOK());
  ASSERT_THROW(packet->isResponseOK(), std::runtime_error);
}

TEST(TxRxParamWritePacket, WriteAndWaitResponse) {
  std::string tx_ret;
  auto packet = std::make_unique<TxRxParamWritePacket>();

  // Packet sendable
  ASSERT_TRUE(packet->isOK());

  packet->setRightGain(100);
  ASSERT_TRUE(packet->getTx(tx_ret));
  ASSERT_EQ(tx_ret, "$00W001E006405*\r");
  ASSERT_TRUE(packet->isWaitingResponse());

  // Can not set new TX until response processed
  ASSERT_FALSE(packet->isOK());
  ASSERT_FALSE(packet->getTx(tx_ret));

  const std::string data_ok = "$00W17*";
  ASSERT_TRUE(packet->setRx(data_ok));
  ASSERT_FALSE(packet->isWaitingResponse());
  ASSERT_TRUE(packet->isOK());

  packet->setLeftGain(100);
  ASSERT_TRUE(packet->getTx(tx_ret));
  ASSERT_EQ(tx_ret, "$00W001F006406*\r");
  ASSERT_TRUE(packet->isWaitingResponse());

  const std::string data_ng = "$00W17/";
  ASSERT_FALSE(packet->setRx(data_ng));
  ASSERT_FALSE(packet->isWaitingResponse());
  ASSERT_TRUE(packet->isOK());
}
