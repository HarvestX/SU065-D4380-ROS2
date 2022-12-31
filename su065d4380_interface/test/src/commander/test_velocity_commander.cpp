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

#include <gmock/gmock.h>
#include <h6x_serial_interface/gmock_port_handler.hpp>
#include <su065d4380_interface/commander/velocity_commander.hpp>

using ::testing::_;
using ::testing::StrEq;
using ::testing::Return;
using ::testing::DoAll;

using namespace std::chrono_literals;  // NOLINT

class TestVelocityCommander : public ::testing::Test
{
protected:
  using PacketHandler = su065d4380_interface::PacketHandler;
  using VelocityCommander = su065d4380_interface::VelocityCommander;
  using RESPONSE_STATE = su065d4380_interface::RESPONSE_STATE;

  VelocityCommander::UniquePtr commander;
  MockPortHandler mock_port_handler;
  PacketHandler::SharedPtr packet_handler;
  virtual void SetUp()
  {
    this->packet_handler = std::make_shared<PacketHandler>(&mock_port_handler);
    this->commander = std::make_unique<VelocityCommander>(packet_handler, 1ms);
  }

  virtual void TearDown() {}
};

TEST_F(TestVelocityCommander, writeRightWheelRpmOK)
{
  EXPECT_CALL(
    this->mock_port_handler,
    writePort(StrEq("$8C01000BB8000056\r"), _)).Times(1);

  ASSERT_EQ(
    this->commander->writeVelocity(
      su065d4380_interface::FLAG_MODE_MOTOR_ON, 3000, 0), RESPONSE_STATE::WAITING_RESPONSE);

  EXPECT_CALL(this->mock_port_handler, getBytesAvailable()).WillRepeatedly(Return(6));

  EXPECT_CALL(this->mock_port_handler, readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$8C06\r"), Return(6)));

  // Reading packet not enqueued yet, so commander still waiting...
  ASSERT_EQ(this->commander->evaluateResponse(), RESPONSE_STATE::WAITING_RESPONSE);

  // Then here you get response
  this->packet_handler->readPortIntoQueue();

  ASSERT_EQ(this->commander->evaluateResponse(), RESPONSE_STATE::OK);
}

TEST_F(TestVelocityCommander, writeRightWheelRpmExplicitNG)
{
  EXPECT_CALL(this->mock_port_handler, writePort(StrEq("$8C01000BB8000056\r"), _)).Times(1);

  ASSERT_EQ(
    this->commander->writeVelocity(
      su065d4380_interface::FLAG_MODE_MOTOR_ON, 3000, 0), RESPONSE_STATE::WAITING_RESPONSE);

  EXPECT_CALL(this->mock_port_handler, getBytesAvailable()).WillRepeatedly(Return(6));

  EXPECT_CALL(this->mock_port_handler, readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$8C07\r"), Return(6)));

  // Reading packet not enqueued yet, so commander still waiting...
  ASSERT_EQ(this->commander->evaluateResponse(), RESPONSE_STATE::WAITING_RESPONSE);

  // Then here you get response
  this->packet_handler->readPortIntoQueue();

  ASSERT_EQ(this->commander->evaluateResponse(), RESPONSE_STATE::ERROR_EXPLICIT_NG);
}
