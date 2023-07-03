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

#include <su065d4380_interface/tx_rx_param_read_packet.hpp>

using namespace su065d4380_interface;  // NOLINT

TEST(TxRxParamReadPack, WriteData) {
  std::string out;
  auto packet = std::make_unique<TxParamReadPacket>();

  ASSERT_FALSE(packet->isOK());
  packet->setRightGain();
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00R001E02*\r");
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  packet->setLeftGain();
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00R001F01*\r");
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  packet->setAccelerationTime();
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00R002074*\r");
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  packet->setDecelerationTime();
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00R002175*\r");
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  packet->setTimeout();
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00R002571*\r");
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  packet->setInputOffDecelerationTime();
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->get(out));
  ASSERT_EQ(out, "$00R002672*\r");
  packet->consume();
}

TEST(TxRxParamReadPacket, ReadResponse) {
  auto packet = std::make_unique<RxParamReadPacket>();

  ASSERT_FALSE(packet->isOK());
  const std::string data_ng = "$00R12/";
  ASSERT_TRUE(packet->set(data_ng));
  ASSERT_TRUE(packet->isOK());
  ASSERT_FALSE(packet->isResponseOK());
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  const std::string data_ok = "$00R006474*";
  ASSERT_TRUE(packet->set(data_ok));
  ASSERT_TRUE(packet->isOK());
  ASSERT_TRUE(packet->isResponseOK());
  ASSERT_EQ(packet->getData(), static_cast<uint16_t>(100));
  packet->consume();

  ASSERT_FALSE(packet->isOK());
  const std::string data_inval = "$00R006474|";    // Invalid CRC
  ASSERT_FALSE(packet->set(data_inval));
  ASSERT_FALSE(packet->isOK());
  ASSERT_THROW(packet->isResponseOK(), std::runtime_error);
}


TEST(TxRxParamReadPacket, WriteAndWaitResponse)
{
  std::string tx_ret;
  auto packet = std::make_unique<TxRxParamReadPacket>();

  // Packet sendable
  ASSERT_TRUE(packet->isOK());

  packet->setRightGain();
  ASSERT_TRUE(packet->getTx(tx_ret));
  ASSERT_EQ(tx_ret, "$00R001E02*\r");
  ASSERT_TRUE(packet->isWaitingResponse());

  // Can not set new Tx until response processed
  ASSERT_FALSE(packet->isOK());
  ASSERT_FALSE(packet->getTx(tx_ret));

  const std::string data_ok = "$00R006474*";
  ASSERT_TRUE(packet->setRx(data_ok));
  ASSERT_FALSE(packet->isWaitingResponse());
  ASSERT_EQ(packet->getData(), 100);
  ASSERT_TRUE(packet->isOK());

  packet->setLeftGain();
  ASSERT_TRUE(packet->getTx(tx_ret));
  ASSERT_EQ(tx_ret, "$00R001F01*\r");
  ASSERT_TRUE(packet->isWaitingResponse());

  const std::string data_ng = "$00R12/";
  ASSERT_FALSE(packet->setRx(data_ng));
  ASSERT_FALSE(packet->isWaitingResponse());
  ASSERT_TRUE(packet->isOK());
}
