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

#include "su065d4380_interface/param_writer_interface.hpp"

namespace su065d4380_interface
{
CallbackReturn ParamWriterInterface::on_init()
{
  this->tx_rx_param_write_packet_ = std::make_unique<TxRxParamWritePacket>();

  return CallbackReturn::SUCCESS;
}

ParamWriterInterface::~ParamWriterInterface()
{
  this->tx_rx_param_write_packet_.reset();
}

void ParamWriterInterface::writeRightGain(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set right gain write packet.");
    return;
  }

  std::string data;
  this->tx_rx_param_write_packet_->setRightGain(val);
  if (!this->tx_rx_param_write_packet_->getTx(data)) {
    RCLCPP_ERROR(this->getLogger(), "Failed to take Tx packet.");
    return;
  }

  this->port_handler_->write(data.data(), data.size());
  while (this->tx_rx_param_write_packet_->isWaitingResponse()) {
    this->read();
  }
}

void ParamWriterInterface::writeLeftGain(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set left gain write packet.");
    return;
  }

  std::string data;
  this->tx_rx_param_write_packet_->setLeftGain(val);
  if (!this->tx_rx_param_write_packet_->getTx(data)) {
    RCLCPP_ERROR(this->getLogger(), "Failed to take Tx packet.");
    return;
  }

  this->port_handler_->write(data.data(), data.size());
  while (this->tx_rx_param_write_packet_->isWaitingResponse()) {
    this->read();
  }
}

void ParamWriterInterface::writeAccelerationTime(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Filed to set acceleration time write packet.");
    return;
  }

  std::string data;
  this->tx_rx_param_write_packet_->setAccelerationTime(val);
  if (!this->tx_rx_param_write_packet_->getTx(data)) {
    RCLCPP_ERROR(this->getLogger(), "Failed to take Tx packet.");
    return;
  }

  this->port_handler_->write(data.data(), data.size());
  while (this->tx_rx_param_write_packet_->isWaitingResponse()) {
    this->read();
  }
}

void ParamWriterInterface::writeDecelerationTime(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set deceleration time write packet.");
    return;
  }

  std::string data;
  this->tx_rx_param_write_packet_->setDecelerationTime(val);
  if (!this->tx_rx_param_write_packet_->getTx(data)) {
    RCLCPP_ERROR(this->getLogger(), "Failed to take Tx packet.");
    return;
  }

  this->port_handler_->write(data.data(), data.size());
  while (this->tx_rx_param_write_packet_->isWaitingResponse()) {
    this->read();
  }
}

void ParamWriterInterface::writeTimeout(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set timeout write packet.");
    return;
  }

  std::string data;
  this->tx_rx_param_write_packet_->setTimeout(val);
  if (!this->tx_rx_param_write_packet_->getTx(data)) {
    RCLCPP_ERROR(this->getLogger(), "Failed to take Tx packet.");
    return;
  }

  this->port_handler_->write(data.data(), data.size());
  while (this->tx_rx_param_write_packet_->isWaitingResponse()) {
    this->read();
  }
}

void ParamWriterInterface::writeInputOffDecelerationTime(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set input off deceleration time write packet.");
    return;
  }

  std::string data;
  this->tx_rx_param_write_packet_->setInputOffDecelerationTime(val);
  if (!this->tx_rx_param_write_packet_->getTx(data)) {
    RCLCPP_ERROR(this->getLogger(), "Failed to take Tx packet.");
    return;
  }

  this->port_handler_->write(data.data(), data.size());
  while (this->tx_rx_param_write_packet_->isWaitingResponse()) {
    this->read();
  }
}

void ParamWriterInterface::readSinglePacket(const std::string & packet)
{
  try {
    if (this->tx_rx_param_write_packet_->setRx(packet)) {
      RCLCPP_INFO(this->getLogger(), "Succeeded to write!");
    } else {
      RCLCPP_ERROR(this->getLogger(), "Failed to write");
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->getLogger(), e.what());
  }
}

const rclcpp::Logger ParamWriterInterface::getLogger() noexcept
{
  return rclcpp::get_logger("ParamWriterInterface");
}
}  // namespace su065d4380_interface
