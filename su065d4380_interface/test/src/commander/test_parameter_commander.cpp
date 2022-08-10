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
#include <su065d4380_interface/commander/parameter_commander.hpp>

using ::testing::_;
using ::testing::StrEq;
using ::testing::SetArrayArgument;
using ::testing::Return;
using ::testing::DoAll;

ACTION_P(StrCpyToArg0, str) {
  strcpy(arg0, str);
}

class MockPortHandler : public su065d4380_interface::PortHandlerBase
{
public:
  MockPortHandler()
  : su065d4380_interface::PortHandlerBase()
  {

  }

  MOCK_METHOD(
    int, readPort, (char *, const int), (const override));
  MOCK_METHOD(
    int, writePort, (const char *, const int), (const override));
};

using namespace std::chrono_literals;

class TestParameterCommander : public ::testing::Test
{
protected:
  std::unique_ptr<su065d4380_interface::ParameterCommander> commander;
  MockPortHandler mock;
  virtual void SetUp()
  {
    this->commander =
      std::make_unique<su065d4380_interface::ParameterCommander>(
      &this->mock, 1ms);
  }

  virtual void TearDown() {}
};

TEST_F(TestParameterCommander, writeRightWheelGainOK) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W001E006405\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_TRUE(
    this->commander->writeRightWheelGain(100));
}

TEST_F(TestParameterCommander, writeRightWheelGainNG) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W001E006405\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_FALSE(
    this->commander->writeRightWheelGain(100));
}

TEST_F(TestParameterCommander, writeRightWheelGainInvalidRange)
{
  ASSERT_FALSE(
    this->commander->writeRightWheelGain(101));

  ASSERT_FALSE(
    this->commander->writeRightWheelGain(-1));
}

TEST_F(TestParameterCommander, writeRightWheelGainInvalidResponse) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W001E006405\r"), _)).Times(1);
  // Return invalid response here
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_THROW(
    this->commander->writeRightWheelGain(100),
    std::runtime_error);
}


TEST_F(TestParameterCommander, writeLeftWheelGainOK) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W001F006406\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_TRUE(
    this->commander->writeLeftWheelGain(100));
}

TEST_F(TestParameterCommander, writeLeftWheelGainNG) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W001F006406\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_FALSE(
    this->commander->writeLeftWheelGain(100));
}

TEST_F(TestParameterCommander, writeLeftWheelGainInvalidRange)
{
  ASSERT_FALSE(
    this->commander->writeLeftWheelGain(101));

  ASSERT_FALSE(
    this->commander->writeLeftWheelGain(-1));
}

TEST_F(TestParameterCommander, writeLeftWheelGainInvalidResponse)
{
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W001F006406\r"), _)).Times(1);
  // Return invalid response here
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_THROW(
    this->commander->writeLeftWheelGain(100),
    std::runtime_error);
}

TEST_F(TestParameterCommander, writeAccTimeOK) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W002001F402\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_TRUE(
    this->commander->writeAccTime(500));
}

TEST_F(TestParameterCommander, writeAccTimeNG) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W002001F402\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_FALSE(
    this->commander->writeAccTime(500));
}

TEST_F(TestParameterCommander, writeAccTimeInvalidRange)
{
  ASSERT_FALSE(
    this->commander->writeAccTime(501));

  ASSERT_FALSE(
    this->commander->writeAccTime(-1));
}

TEST_F(TestParameterCommander, writeAccTimeInvalidResponse)
{
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W002001F402\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_THROW(
    this->commander->writeAccTime(500), std::runtime_error);
}

TEST_F(TestParameterCommander, writeDecTimeOK) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W002101F403\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_TRUE(
    this->commander->writeDecTime(500));
}

TEST_F(TestParameterCommander, writeDecTimeNG) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W002101F403\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_FALSE(
    this->commander->writeDecTime(500));
}

TEST_F(TestParameterCommander, writeDecTimeInvalidRange) {
  ASSERT_FALSE(
    this->commander->writeDecTime(501));

  ASSERT_FALSE(
    this->commander->writeDecTime(-1));
}

TEST_F(TestParameterCommander, writeDecTimeInvalidResponse)
{
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W002101F403\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_THROW(
    this->commander->writeDecTime(500), std::runtime_error);
}

TEST_F(TestParameterCommander, writeTimeoutOK) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W0025000571\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_TRUE(
    this->commander->writeTimeout(5));
}

TEST_F(TestParameterCommander, writeTimeoutNG) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W0025000571\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_FALSE(
    this->commander->writeTimeout(5));
}

TEST_F(TestParameterCommander, writeTimeoutInvalidRange)
{
  ASSERT_FALSE(
    this->commander->writeTimeout(6));

  ASSERT_FALSE(
    this->commander->writeTimeout(-1));
}

TEST_F(TestParameterCommander, writeTimeoutInvalidResponse) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W0025000571\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_THROW(
    this->commander->writeTimeout(5), std::runtime_error);
}

TEST_F(TestParameterCommander, writeDecWithTimeoutOK) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W002601F404\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17*\r"), Return(8)));

  ASSERT_TRUE(
    this->commander->writeDecWithTimeout(500));
}

TEST_F(TestParameterCommander, writeDecWithTimeoutNG) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W002601F404\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17/\r"), Return(8)));

  ASSERT_FALSE(
    this->commander->writeDecWithTimeout(500));
}

TEST_F(TestParameterCommander, writeDecWithTimeoutInvalidRange)
{
  ASSERT_FALSE(
    this->commander->writeDecWithTimeout(501));

  ASSERT_FALSE(
    this->commander->writeDecWithTimeout(-1));
}

TEST_F(TestParameterCommander, writeDecWithTimeoutInvalidResponse) {
  EXPECT_CALL(
    mock,
    writePort(StrEq("$00W002601F404\r"), _)).Times(1);
  EXPECT_CALL(
    mock,
    readPort(_, _))
  .WillRepeatedly(DoAll(StrCpyToArg0("$00W17x\r"), Return(8)));

  ASSERT_THROW(
    this->commander->writeDecWithTimeout(500), std::runtime_error);
}
