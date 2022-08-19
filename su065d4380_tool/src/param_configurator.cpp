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

#include "su065d4380_tool/param_configurator.hpp"

namespace su065d4380_tool
{
ParamConfigurator::ParamConfigurator(const rclcpp::NodeOptions & options)
: rclcpp::Node("param_configurator", options),
  dev_(this->declare_parameter("dev", "/dev/ttyUSB0"))
{
  RCLCPP_INFO(
    this->get_logger(),
    "Selected device: %s",
    this->dev_.c_str());

  this->port_handler_ =
    std::make_shared<su065d4380_interface::PortHandler>(this->dev_);
  this->port_handler_->openPort();

  this->commander_ =
    std::make_shared<su065d4380_interface::ParameterCommander>(
    this->port_handler_.get());
}

ParamConfigurator::~ParamConfigurator()
{
  this->port_handler_->closePort();
}

void ParamConfigurator::readAll() const
{
  for (int i = 0; i < static_cast<int>(CONFIGURABLE_PARAM::END); ++i) {
    this->read(static_cast<CONFIGURABLE_PARAM>(i));
  }
}

void ParamConfigurator::read(const CONFIGURABLE_PARAM & target) const
{
  int val;
  switch (target) {
    case CONFIGURABLE_PARAM::RIGHT_WHEEL_GAIN:
      val = this->commander_->readRightWheelGain();
      break;
    case CONFIGURABLE_PARAM::LEFT_WHEEL_GAIN:
      val = this->commander_->readLeftWheelGain();
      break;
    case CONFIGURABLE_PARAM::ACC_TIME:
      val = this->commander_->readAccTime();
      break;
    case CONFIGURABLE_PARAM::DEC_TIME:
      val = this->commander_->readDecTime();
      break;
    case CONFIGURABLE_PARAM::TIMEOUT:
      val = this->commander_->readTimeout();
      break;
    case CONFIGURABLE_PARAM::DEC_WITH_TIMEOUT:
      val = this->commander_->readDecWithTimeout();
      break;
    case CONFIGURABLE_PARAM::END:
    // fall through
    default:
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid configuration target");
      return;
  }

  if (val < 0) {
    // error
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "%s: %d", this->getParamName(target).c_str(), val);
}

void ParamConfigurator::write(
  const CONFIGURABLE_PARAM & target, const int val) const
{
  bool res = false;
  switch (target) {
    case CONFIGURABLE_PARAM::RIGHT_WHEEL_GAIN:
      res = this->commander_->writeRightWheelGain(val);
      break;
    case CONFIGURABLE_PARAM::LEFT_WHEEL_GAIN:
      res = this->commander_->writeLeftWheelGain(val);
      break;
    case CONFIGURABLE_PARAM::ACC_TIME:
      res = this->commander_->writeAccTime(val);
      break;
    case CONFIGURABLE_PARAM::DEC_TIME:
      break;
    case CONFIGURABLE_PARAM::TIMEOUT:
      res = this->commander_->writeTimeout(val);
      break;
    case CONFIGURABLE_PARAM::DEC_WITH_TIMEOUT:
      res = this->commander_->writeDecWithTimeout(val);
      break;
    case CONFIGURABLE_PARAM::END:
    // fall through
    default:
      RCLCPP_ERROR(
        this->get_logger(),
        "Invalid configuration target");
      return;
  }

  if (res) {
    RCLCPP_INFO(
      this->get_logger(),
      "Parameter successfully configured!");
  } else {
    RCLCPP_ERROR(
      this->get_logger(),
      "Configuration Failed");
  }
}

const std::string ParamConfigurator::getParamName(
  const CONFIGURABLE_PARAM & param)
{
  switch (param) {
    case CONFIGURABLE_PARAM::RIGHT_WHEEL_GAIN:
      return "Right Wheel Gain";
    case CONFIGURABLE_PARAM::LEFT_WHEEL_GAIN:
      return "Left Wheel Gain";
    case CONFIGURABLE_PARAM::ACC_TIME:
      return "Acc Time";
      break;
    case CONFIGURABLE_PARAM::DEC_TIME:
      return "Dec Time";
      break;
    case CONFIGURABLE_PARAM::TIMEOUT:
      return "Timeout";
      break;
    case CONFIGURABLE_PARAM::DEC_WITH_TIMEOUT:
      return "Dec With Timeout";
      break;
    case CONFIGURABLE_PARAM::END:
    // fall through
    default:
      return "";
  }
}
}  // namespace su065d4380_tool


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto configurator =
    std::make_shared<su065d4380_tool::ParamConfigurator>(rclcpp::NodeOptions());

  configurator->readAll();
  return EXIT_SUCCESS;
}
