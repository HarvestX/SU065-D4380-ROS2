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
#include <su065d4380_interface/commander/info_commander.hpp>

using ::testing::_;
using ::testing::StrEq;
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

  MOCK_METHOD(size_t, getBytesAvailable, (), (const override));
  MOCK_METHOD(size_t, readPort, (char *, const size_t), (const override));
  MOCK_METHOD(
    size_t, writePort,
    (const char *, const size_t), (const override));
};

using namespace std::chrono_literals;

class TestInfoCommander : public ::testing::Test
{
protected:
  std::shared_ptr<su065d4380_interface::PacketHandler> packet_handler;
  std::unique_ptr<su065d4380_interface::InfoCommander> commander;
  MockPortHandler mock_port_handler;
  virtual void SetUp()
  {
    this->packet_handler =
      std::make_shared<su065d4380_interface::PacketHandler>(
      &mock_port_handler);
    this->commander =
      std::make_unique<su065d4380_interface::InfoCommander>(
      this->packet_handler, 1ms);
  }

  virtual void TearDown() {}
};


TEST_F(TestInfoCommander, readAllOK)
{
  EXPECT_CALL(
    mock_port_handler,
    getBytesAvailable())
  .WillRepeatedly(Return(5 * 14));
  // Return invalid response here
  EXPECT_CALL(
    mock_port_handler,
    readPort(_, _))
  .WillRepeatedly(
    DoAll(
      StrCpyToArg0(
        "$A10100F83028\r"
        "$A20100F8302B\r"
        "$A3000200085C\r"
        "$A43FFF000125\r"
        "$A512D3000024\r"), Return(5 * 14)));

  this->packet_handler->readPortIntoQueue();
  this->commander->evaluateResponse();

  uint8_t mode = 0;
  int16_t rpm = 0;
  ASSERT_EQ(
    this->commander->readRightRpm(mode, rpm),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_TRUE(mode &= su065d4380_interface::FLAG_MODE_MOTOR_ON);
  ASSERT_EQ(rpm, -2000);

  mode = 0;
  rpm = 0;
  ASSERT_EQ(
    this->commander->readLeftRpm(mode, rpm),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_TRUE(mode &= su065d4380_interface::FLAG_MODE_MOTOR_ON);
  ASSERT_EQ(rpm, -2000);

  su065d4380_interface::DriverState driver_state;
  ASSERT_EQ(
    this->commander->readDriverState(driver_state),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_TRUE(driver_state.hasError());
  ASSERT_EQ(
    driver_state.getErrorState(),
    su065d4380_interface::ERROR_STATE::SENSOR_ERROR);

  int16_t right_encoder, left_encoder;
  ASSERT_EQ(
    this->commander->readEncoderData(right_encoder, left_encoder),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_EQ(
    right_encoder, 16383);
  ASSERT_EQ(left_encoder, 1);

  float voltage;
  ASSERT_EQ(
    this->commander->readVoltage(voltage),
    su065d4380_interface::RESPONSE_STATE::OK);
  ASSERT_FLOAT_EQ(voltage, 48.19);
}
