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

  this->tx_rx_param_write_packet_->setRightGain(val);
  if (!this->flashAndWaitResponse()) {return;}

  if (this->read_status_) {
    RCLCPP_INFO(this->getLogger(), "Right gain %d.", val);
  } else {
    RCLCPP_ERROR(this->getLogger(), "Failed to set right gain.");
  }
}

void ParamWriterInterface::writeLeftGain(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set left gain write packet.");
    return;
  }

  this->tx_rx_param_write_packet_->setLeftGain(val);
  if (!this->flashAndWaitResponse()) {return;}

  if (this->read_status_) {
    RCLCPP_INFO(this->getLogger(), "Left gain %d.", val);
  } else {
    RCLCPP_ERROR(this->getLogger(), "Failed to set left gain.");
  }
}

void ParamWriterInterface::writeAccelerationTime(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Filed to set acceleration time write packet.");
    return;
  }

  this->tx_rx_param_write_packet_->setAccelerationTime(val);
  if (!this->flashAndWaitResponse()) {return;}

  if (this->read_status_) {
    RCLCPP_INFO(this->getLogger(), "Acceleration time %d.", val);
  } else {
    RCLCPP_ERROR(this->getLogger(), "Failed to set acceleration time.");
  }
}

void ParamWriterInterface::writeDecelerationTime(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set deceleration time write packet.");
    return;
  }

  this->tx_rx_param_write_packet_->setDecelerationTime(val);
  if (!this->flashAndWaitResponse()) {return;}

  if (this->read_status_) {
    RCLCPP_INFO(this->getLogger(), "Deceleration time %d.", val);
  } else {
    RCLCPP_ERROR(this->getLogger(), "Failed to set deceleration time.");
  }
}

void ParamWriterInterface::writeTimeout(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set timeout write packet.");
    return;
  }

  this->tx_rx_param_write_packet_->setTimeout(val);
  if (!this->flashAndWaitResponse()) {return;}

  if (this->read_status_) {
    RCLCPP_INFO(this->getLogger(), "Timeout %d.", val);
  } else {
    RCLCPP_ERROR(this->getLogger(), "Failed to write timeout.");
  }
}

void ParamWriterInterface::writeInputOffDecelerationTime(const uint16_t val)
{
  if (!this->tx_rx_param_write_packet_->isOK()) {
    RCLCPP_ERROR(this->getLogger(), "Failed to set input off deceleration time write packet.");
    return;
  }

  this->tx_rx_param_write_packet_->setInputOffDecelerationTime(val);
  if (!this->flashAndWaitResponse()) {return;}

  if (this->read_status_) {
    RCLCPP_INFO(this->getLogger(), "Input Off Deceleration Time %d.", val);
  } else {
    RCLCPP_ERROR(this->getLogger(), "Failed to write input off deceleration time.");
  }
}

void ParamWriterInterface::readSinglePacket(const std::string & packet)
{
  try {
    this->read_status_ = this->tx_rx_param_write_packet_->setRx(packet);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->getLogger(), e.what());
    this->read_status_ = false;
  }
}

bool ParamWriterInterface::flashAndWaitResponse()
{
  std::string data;
  if (!this->tx_rx_param_write_packet_->getTx(data)) {
    RCLCPP_ERROR(this->getLogger(), "Failed to take Tx packet.");
    return false;
  }

  this->port_handler_->write(data.data(), data.size());
  while (this->tx_rx_param_write_packet_->isWaitingResponse()) {
    if (!rclcpp::ok()) {
      return false;
    }
    this->read();
  }

  return true;
}

const rclcpp::Logger ParamWriterInterface::getLogger() noexcept
{
  return rclcpp::get_logger("ParamWriterInterface");
}
}  // namespace su065d4380_interface
