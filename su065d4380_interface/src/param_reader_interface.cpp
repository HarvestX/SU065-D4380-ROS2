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

#include "su065d4380_interface/param_reader_interface.hpp"

namespace su065d4380_interface
{
CallbackReturn ParamReaderInterface::on_init()
{
  this->tx_rx_param_read_packet_ = std::make_unique<TxRxParamReadPacket>();

  return CallbackReturn::SUCCESS;
}

ParamReaderInterface::~ParamReaderInterface()
{
  this->tx_rx_param_read_packet_.reset();
}

void ParamReaderInterface::readRightGain()
{
  if (!this->tx_rx_param_read_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set right gain read packet.");
    return;
  }

  this->tx_rx_param_read_packet_->setRightGain();
  if (!this->flashAndWaitResponse()) {
    return;
  }

  RCLCPP_INFO(this->getLogger(), "Right gain %d.", this->tx_rx_param_read_packet_->getData());
}

void ParamReaderInterface::readLeftGain()
{
  if (!this->tx_rx_param_read_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set left gain read packet.");
    return;
  }

  this->tx_rx_param_read_packet_->setLeftGain();
  if (!this->flashAndWaitResponse()) {
    return;
  }

  RCLCPP_INFO(this->getLogger(), "Left gain %d.", this->tx_rx_param_read_packet_->getData());
}

void ParamReaderInterface::readAccelerationTime()
{
  if (!this->tx_rx_param_read_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set acceleration time read packet.");
    return;
  }

  this->tx_rx_param_read_packet_->setAccelerationTime();
  if (!this->flashAndWaitResponse()) {
    return;
  }

  RCLCPP_INFO(
    this->getLogger(), "Acceleration time %d.",
    this->tx_rx_param_read_packet_->getData());
}

void ParamReaderInterface::readDecelerationTime()
{
  if (!this->tx_rx_param_read_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set deceleration time read packet.");
    return;
  }

  this->tx_rx_param_read_packet_->setDecelerationTime();
  if (!this->flashAndWaitResponse()) {
    return;
  }

  RCLCPP_INFO(
    this->getLogger(), "Deceleration time %d.",
    this->tx_rx_param_read_packet_->getData());
}

void ParamReaderInterface::readTimeout()
{
  if (!this->tx_rx_param_read_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set timeout read packet.");
    return;
  }

  this->tx_rx_param_read_packet_->setTimeout();
  if (!this->flashAndWaitResponse()) {
    return;
  }

  RCLCPP_INFO(this->getLogger(), "Timeout %d.", this->tx_rx_param_read_packet_->getData());
}

void ParamReaderInterface::readInputOffDecelerationTime()
{
  if (!this->tx_rx_param_read_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set input off deceleration time read packet.");
    return;
  }

  std::string data;
  this->tx_rx_param_read_packet_->setInputOffDecelerationTime();
  if (!this->flashAndWaitResponse()) {
    return;
  }

  RCLCPP_INFO(
    this->getLogger(), "Input off deceleration time %d.",
    this->tx_rx_param_read_packet_->getData());
}

void ParamReaderInterface::readSinglePacket(const std::string & packet)
{
  try {
    this->tx_rx_param_read_packet_->setRx(packet);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->getLogger(), e.what());
  }
}

bool ParamReaderInterface::flashAndWaitResponse()
{
  std::string data;
  if (!this->tx_rx_param_read_packet_->getTx(data)) {
    RCLCPP_ERROR(this->getLogger(), "Failed to take Tx packet.");
    return false;
  }

  this->port_handler_->write(data.data(), data.size());
  while (this->tx_rx_param_read_packet_->isWaitingResponse()) {
    if (!rclcpp::ok()) {
      return false;
    }

    this->read();
  }

  return true;
}

const rclcpp::Logger ParamReaderInterface::getLogger() noexcept
{
  return rclcpp::get_logger("ParamReaderInterface");
}
}  // namespace su065d4380_interface
