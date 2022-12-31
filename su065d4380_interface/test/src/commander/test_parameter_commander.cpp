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
#include <su065d4380_interface/commander/parameter_commander.hpp>

using ::testing::_;
using ::testing::StrEq;
using ::testing::Return;
using ::testing::DoAll;

using namespace std::chrono_literals;  // NOLINT

class TestParameterCommander : public ::testing::Test
{
protected:
  std::unique_ptr<su065d4380_interface::ParameterCommander> commander;
  MockPortHandler mock_port_handler;
  virtual void SetUp()
  {
    auto packet_handler =
      std::make_shared<su065d4380_interface::PacketHandler>(
      &mock_port_handler);
    this->commander =
      std::make_unique<su065d4380_interface::ParameterCommander>(
      packet_handler, 1ms);
  }

  virtual void TearDown() {}
};

TEST_F(TestParameterCommander, writeRightWheelGainOK) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W001E006405\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeRightWheelGain(100),
    su065d4380_interface::RESPONSE_STATE::OK);
}

TEST_F(TestParameterCommander, writeRightWheelGainNG) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W001E006405\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeRightWheelGain(100),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, writeRightWheelGainInvalidRange)
{
  ASSERT_EQ(
    this->commander->writeRightWheelGain(101),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);

  ASSERT_EQ(
    this->commander->writeRightWheelGain(-1),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);
}

TEST_F(TestParameterCommander, writeRightWheelGainInvalidResponse) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W001E006405\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  // Return invalid response here
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeRightWheelGain(100),
    su065d4380_interface::RESPONSE_STATE::ERROR_UNKNOWN);
}


TEST_F(TestParameterCommander, writeLeftWheelGainOK) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W001F006406\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));


  ASSERT_EQ(
    this->commander->writeLeftWheelGain(100),
    su065d4380_interface::RESPONSE_STATE::OK);
}

TEST_F(TestParameterCommander, writeLeftWheelGainNG) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W001F006406\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeLeftWheelGain(100),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, writeLeftWheelGainInvalidRange)
{
  ASSERT_EQ(
    this->commander->writeLeftWheelGain(101),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);

  ASSERT_EQ(
    this->commander->writeLeftWheelGain(-1),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);
}

TEST_F(TestParameterCommander, writeLeftWheelGainInvalidResponse)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W001F006406\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  // Return invalid response here
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeLeftWheelGain(100),
    su065d4380_interface::RESPONSE_STATE::ERROR_UNKNOWN);
}

TEST_F(TestParameterCommander, writeAccTimeOK) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W002001F402\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeAccTime(500),
    su065d4380_interface::RESPONSE_STATE::OK);
}

TEST_F(TestParameterCommander, writeAccTimeNG) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W002001F402\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeAccTime(500),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, writeAccTimeInvalidRange)
{
  ASSERT_EQ(
    this->commander->writeAccTime(501),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);
  ASSERT_EQ(
    this->commander->writeAccTime(-1),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);
}

TEST_F(TestParameterCommander, writeAccTimeInvalidResponse)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W002001F402\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeAccTime(500),
    su065d4380_interface::RESPONSE_STATE::ERROR_UNKNOWN);
}

TEST_F(TestParameterCommander, writeDecTimeOK) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W002101F403\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeDecTime(500),
    su065d4380_interface::RESPONSE_STATE::OK);
}

TEST_F(TestParameterCommander, writeDecTimeNG) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W002101F403\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeDecTime(500),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, writeDecTimeInvalidRange) {
  ASSERT_EQ(
    this->commander->writeDecTime(501),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);

  ASSERT_EQ(
    this->commander->writeDecTime(-1),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);
}

TEST_F(TestParameterCommander, writeDecTimeInvalidResponse)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W002101F403\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeDecTime(500),
    su065d4380_interface::RESPONSE_STATE::ERROR_UNKNOWN);
}

TEST_F(TestParameterCommander, writeTimeoutOK) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W0025000571\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeTimeout(5),
    su065d4380_interface::RESPONSE_STATE::OK);
}

TEST_F(TestParameterCommander, writeTimeoutNG) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W0025000571\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeTimeout(5),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, writeTimeoutInvalidRange)
{
  ASSERT_EQ(
    this->commander->writeTimeout(6),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);

  ASSERT_EQ(
    this->commander->writeTimeout(-1),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);
}

TEST_F(TestParameterCommander, writeTimeoutInvalidResponse) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W0025000571\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeTimeout(5),
    su065d4380_interface::RESPONSE_STATE::ERROR_UNKNOWN);
}

TEST_F(TestParameterCommander, writeDecWithTimeoutOK) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W002601F404\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeDecWithTimeout(500),
    su065d4380_interface::RESPONSE_STATE::OK);
}

TEST_F(TestParameterCommander, writeDecWithTimeoutNG) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W002601F404\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeDecWithTimeout(500),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, writeDecWithTimeoutInvalidRange)
{
  ASSERT_EQ(
    this->commander->writeDecWithTimeout(501),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);

  ASSERT_EQ(
    this->commander->writeDecWithTimeout(-1),
    su065d4380_interface::RESPONSE_STATE::ERROR_INVALID_INPUT);
}

TEST_F(TestParameterCommander, writeDecWithTimeoutInvalidResponse) {
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00W002601F404\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_EQ(
    this->commander->writeDecWithTimeout(500),
    su065d4380_interface::RESPONSE_STATE::ERROR_UNKNOWN);
}


TEST_F(TestParameterCommander, readRightWheelGainOK)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R001E02\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R006474*\r"), Return(12)));

  int ret;
  ASSERT_EQ(
    this->commander->readRightWheelGain(ret),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_EQ(ret, 100);
}

TEST_F(TestParameterCommander, readRightWheelGainNG)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R001E02\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R12/\r"), Return(8)));

  int ret;
  ASSERT_EQ(
    this->commander->readRightWheelGain(ret),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, readLeftWheelGainOK)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R001F01\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R006474*\r"), Return(12)));

  int ret;
  ASSERT_EQ(
    this->commander->readLeftWheelGain(ret),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_EQ(ret, 100);
}

TEST_F(TestParameterCommander, readLeftWheelGainNG)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R001F01\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R12/\r"), Return(8)));

  int ret;
  ASSERT_EQ(
    this->commander->readLeftWheelGain(ret),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, readAccTimeOK)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R002074\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R01F405*\r"), Return(12)));

  int ret;
  ASSERT_EQ(
    this->commander->readAccTime(ret),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_EQ(ret, 500);
}

TEST_F(TestParameterCommander, readAccTimeNG)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R002074\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R12/\r"), Return(8)));

  int ret;
  ASSERT_EQ(
    this->commander->readAccTime(ret),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, readDecTimeOK)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R002175\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R01F405*\r"), Return(12)));

  int ret;
  ASSERT_EQ(
    this->commander->readDecTime(ret),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_EQ(ret, 500);
}

TEST_F(TestParameterCommander, readDecTimeNG)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R002175\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R12/\r"), Return(8)));

  int ret;
  ASSERT_EQ(
    this->commander->readDecTime(ret),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, readTimeoutOK)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R002571\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R000076*\r"), Return(12)));

  int ret;
  ASSERT_EQ(
    this->commander->readTimeout(ret),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_EQ(ret, 0);
}

TEST_F(TestParameterCommander, readTimeoutNG)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R002571\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R12/\r"), Return(8)));

  int ret;
  ASSERT_EQ(
    this->commander->readTimeout(ret),
    su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}

TEST_F(TestParameterCommander, readDecWithTimeoutOK)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R002672\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R01F405*\r"), Return(12)));

  int ret;
  ASSERT_EQ(
    this->commander->readDecWithTimeout(ret),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_EQ(ret, 500);
}

TEST_F(TestParameterCommander, readDecWithTimeoutNG)
{
  EXPECT_CALL(
    mock_port_handler,
    writePort(StrEq("$00R002672\r"), _)).Times(1);
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(8));
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00R12/\r"), Return(8)));

  int ret;
  ASSERT_EQ(
    this->commander->readDecWithTimeout(
      ret), su065d4380_interface::RESPONSE_STATE::ERROR_EXPLICIT_NG);
}
