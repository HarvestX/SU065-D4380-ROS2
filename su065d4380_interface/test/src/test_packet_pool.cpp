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

#include <gtest/gtest.h>
#include <su065d4380_interface/packet_pool.hpp>

class TestPacketPool : public ::testing::Test
{
protected:
  std::unique_ptr<su065d4380_interface::PacketPool> pool;
  virtual void SetUp()
  {
    this->pool =
      std::make_unique<su065d4380_interface::PacketPool>();
  }
};

TEST_F(TestPacketPool, enqueueFineCommands) {
  const std::string velocity_response = "$8C07\r";
  const std::string info_response = "$A10100F83028\r";
  const std::string param_response1 = "$00W17*\r";
  const std::string param_response2 = "$00R003277*\r";

  this->pool->enqueue(
    velocity_response +
    info_response +
    param_response1 +
    param_response2);

  std::string packet = "";

  using PT = su065d4380_interface::PacketPool::PACKET_TYPE;
  ASSERT_TRUE(this->pool->takePacket(PT::VELOCITY, packet));
  EXPECT_EQ(packet, velocity_response);

  ASSERT_TRUE(this->pool->takePacket(PT::INFO, packet));
  EXPECT_EQ(packet, info_response);

  ASSERT_TRUE(this->pool->takePacket(PT::PARAM, packet));
  EXPECT_EQ(packet, param_response1);

  ASSERT_TRUE(this->pool->takePacket(PT::PARAM, packet));
  EXPECT_EQ(packet, param_response2);

  ASSERT_FALSE(this->pool->takePacket(PT::VELOCITY, packet));
  ASSERT_FALSE(this->pool->takePacket(PT::INFO, packet));
  ASSERT_FALSE(this->pool->takePacket(PT::PARAM, packet));
  ASSERT_FALSE(this->pool->takePacket(PT::PARAM, packet));
}

TEST_F(TestPacketPool, enqueueFineSeparatedCommands)
{
  std::string packet;

  const std::string velocity_response = "$8C07\r";
  // 1st part of '$A10100F83028\r'
  const std::string info_response_part1 = "$A10100F";
  const std::string command1 = velocity_response + info_response_part1;

  using PT = su065d4380_interface::PacketPool::PACKET_TYPE;
  this->pool->enqueue(command1);
  ASSERT_TRUE(this->pool->takePacket(PT::VELOCITY, packet));
  EXPECT_EQ(packet, velocity_response);
  ASSERT_FALSE(this->pool->takePacket(PT::INFO, packet));
  EXPECT_EQ(packet, "");

  // 2nd part of '$A10100F83028\r'
  const std::string info_response_part2 = "83028\r";
  const std::string param_response = "$00W17*\r";
  const std::string command2 = info_response_part2 + param_response;

  this->pool->enqueue(command2);
  ASSERT_TRUE(this->pool->takePacket(PT::INFO, packet));
  EXPECT_EQ(packet, info_response_part1 + info_response_part2);
  ASSERT_TRUE(this->pool->takePacket(PT::PARAM, packet));
  EXPECT_EQ(packet, param_response);
}

TEST_F(TestPacketPool, enqueueFineInnerCommands)
{
  std::string packet;

  const std::string info_response = "$A30000000056\r";
  const std::string command = "$" + info_response;

  using PT = su065d4380_interface::PacketPool::PACKET_TYPE;
  this->pool->enqueue(command);
  ASSERT_TRUE(this->pool->takePacket(PT::INFO, packet));
  EXPECT_EQ(packet, info_response);
}

TEST_F(TestPacketPool, enqueueInitiallyGarbageContainedFineCommands) {
  const std::string prefix_garbage = "garbage";
  const std::string param_response = "$00R003277*\r";

  this->pool->enqueue(prefix_garbage + param_response);

  std::string packet;

  using PT = su065d4380_interface::PacketPool::PACKET_TYPE;
  ASSERT_TRUE(this->pool->takePacket(PT::PARAM, packet));
  EXPECT_EQ(packet, param_response);
}
