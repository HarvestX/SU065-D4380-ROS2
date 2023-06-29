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

#include <gtest/gtest.h>

#include <su065d4380_interface/rx_info_packet.hpp>

using namespace su065d4380_interface;  // NOLINT

TEST(RxInfoPacket, RxLeftVelPacket) {
  const auto left_packet = std::make_unique<RxLeftVelPacket>();
  const std::string data = "$A10100F83028";
  ASSERT_FALSE(left_packet->isOK());
  ASSERT_TRUE(left_packet->set(data));
  ASSERT_TRUE(left_packet->isOK());
  ASSERT_EQ(left_packet->getMode(), 1);
  ASSERT_EQ(left_packet->getRPM(), -2000);
  left_packet->consume();
  ASSERT_FALSE(left_packet->isOK());
}

TEST(RxInfoPacket, RxRightVelPacket) {
  const auto right_packet = std::make_unique<RxRightVelPacket>();
  const std::string data = "$A20100F8302B";
  ASSERT_TRUE(right_packet->set(data));
  ASSERT_EQ(right_packet->getMode(), 1);
  ASSERT_EQ(right_packet->getRPM(), -2000);
}

TEST(RxInfoPacket, RxDrvPacket) {
  const auto driver_info_packet = std::make_unique<RxDrvPacket>();
  const std::string data = "$A3000200085C";
  ASSERT_TRUE(driver_info_packet->set(data));
  ASSERT_EQ(driver_info_packet->getDriverState(), 0x0002);
  ASSERT_EQ(driver_info_packet->getErrorState(), 0x0008);
}

TEST(RxInfoPacket, RxEncPacket) {
  const auto encoder_packet = std::make_unique<RxEncPacket>();
  const std::vector<std::string> data_list = {
    "$A40FFF000126",
    "$A41FFF000127",
    "$A42FFF000124",
    "$A43FFF000125",
    "$A44FFF000122"
  };

  // Encoder should be ready for all the time.
  ASSERT_TRUE(encoder_packet->isOK());

  ASSERT_TRUE(encoder_packet->set(data_list.at(0)));
  ASSERT_EQ(encoder_packet->getRightEncoder(), 0x0FFF);
  ASSERT_EQ(encoder_packet->getLeftEncoder(), 0x0001);

  ASSERT_TRUE(encoder_packet->set(data_list.at(1)));
  ASSERT_EQ(encoder_packet->getRightEncoder(), 0x0FFF + 0x1FFF);
  ASSERT_EQ(encoder_packet->getLeftEncoder(), 0x0001 + 0x0001);

  ASSERT_TRUE(encoder_packet->set(data_list.at(2)));
  ASSERT_EQ(encoder_packet->getRightEncoder(), 0x0FFF + 0x1FFF + 0x2FFF);
  ASSERT_EQ(encoder_packet->getLeftEncoder(), 0x0001 + 0x0001 + 0x0001);

  ASSERT_TRUE(encoder_packet->set(data_list.at(3)));
  ASSERT_EQ(encoder_packet->getRightEncoder(), 0x0FFF + 0x1FFF + 0x2FFF + 0x3FFF);
  ASSERT_EQ(encoder_packet->getLeftEncoder(), 0x0001 + 0x0001 + 0x0001 + 0x0001);

  ASSERT_TRUE(encoder_packet->set(data_list.at(4)));
  ASSERT_EQ(encoder_packet->getRightEncoder(), 0x0FFF + 0x1FFF + 0x2FFF + 0x3FFF + 0x4FFF);
  ASSERT_EQ(encoder_packet->getLeftEncoder(), 0x0001 + 0x0001 + 0x0001 + 0x0001 + 0x0001);

  encoder_packet->reset();
  ASSERT_EQ(encoder_packet->getRightEncoder(), 0);
  ASSERT_EQ(encoder_packet->getLeftEncoder(), 0);
}

TEST(RxInfoPacket, RxVolPacket) {
  const auto voltage_packet = std::make_unique<RxVolPacket>();
  const std::string data = "$A512D3000024";
  ASSERT_TRUE(voltage_packet->set(data));
  ASSERT_EQ(voltage_packet->getVoltage(), 48.19);
}
