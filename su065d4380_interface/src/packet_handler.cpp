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

#include "su065d4380_interface/packet_handler.hpp"


namespace su065d4380_interface
{
PacketHandler::PacketHandler(std::unique_ptr<PortHandler> port_handler)
: port_handler_(std::move(port_handler)),
  driver_state_(std::make_unique<info_packet::DriverState>()),
  queue_vel_rx(std::make_unique<std::queue<std::string>>()),
  queue_inf_rx(std::make_unique<std::queue<std::string>>())
{
  this->velcom_state_ = VELCOM_STATE::READY;
}


bool PacketHandler::sendVelocityCommand(
  const uint16_t mode,
  const int32_t right_rpm,
  const int32_t left_rpm)
{
  if (this->velcom_state_ == VELCOM_STATE::WAITING_RESPONSE) {
    RCLCPP_WARN(
      this->logger_,
      "Command not sent. "
      "Waiting for the response from previous packet.");
    return false;
  }

  // TODO: Check rpm < 3000

  std::string send_data;
  velocity_packet::setPacket(
    mode, right_rpm, left_rpm, send_data);
  this->port_handler_->writePort(send_data.data(), send_data.size());
  this->velcom_state_ = VELCOM_STATE::WAITING_RESPONSE;

  RCLCPP_INFO(
    logger_, "Send: %s", send_data.c_str());

  return true;
}

void PacketHandler::recvCommand()
{
  if (this->port_handler_->getBytesAvailable() <
    static_cast<int>(velocity_packet::RX_PACKET_SIZE))
  {
    return;
  }

  char buf[100];
  this->port_handler_->readPort(buf, sizeof(buf));
  this->enqueueCommands(std::string(buf));
  this->evaluateCommands();
}

void PacketHandler::enqueueCommands(const std::string & packet_chunk)
{
  const char delimiter = '\r';
  const char command_symbol = '$';
  std::stringstream packet_chunk_ss(packet_chunk);

  while (packet_chunk_ss.good()) {
    std::string packet;
    std::getline(packet_chunk_ss, packet, delimiter);
    if (packet.size() == 0) {
      continue;
    }
    if (packet.at(0) != command_symbol) {
      continue;
    }

    // Add 1 for removing CR
    switch (packet.size() + 1) {
      case velocity_packet::RX_PACKET_SIZE:
        this->queue_vel_rx->emplace(packet);
        break;
      case info_packet::RX_PACKET_SIZE:
        if (!info_packet::checkSum(packet)) {
          break;
        }
        this->queue_inf_rx->emplace(packet);
        break;
      default:
        break;
    }
  }
}

void PacketHandler::evaluateCommands()
{
  while (!this->queue_vel_rx->empty()) {
    const std::string packet = this->queue_vel_rx->front();
    this->queue_vel_rx->pop();
    try {
      if (velocity_packet::isOK(packet)) {
        RCLCPP_INFO(
          this->logger_,
          "Velocity command accepted.");
      } else {
        RCLCPP_ERROR(
          this->logger_,
          "Failed to sent velocity command");
      }
      this->velcom_state_ = VELCOM_STATE::READY;
    } catch (std::runtime_error & e) {
      RCLCPP_WARN(this->logger_, e.what());
    }
  }

  while (!this->queue_inf_rx->empty()) {
    const std::string packet = this->queue_inf_rx->front();
    this->queue_inf_rx->pop();

    const auto type = info_packet::getCommandType(packet);
    if (type == info_packet::COMMAND_TYPE::RIGHT_WHEEL) {
      this->right_rpm_ = info_packet::getRPM(packet);
    }
    if (type == info_packet::COMMAND_TYPE::LEFT_WHEEL) {
      this->left_rpm_ = info_packet::getRPM(packet);
    }
    if (type == info_packet::COMMAND_TYPE::DRIVER_STATE) {
      info_packet::getDriverState(packet, this->driver_state_);
      if (this->driver_state_->has_error) {
        RCLCPP_ERROR(
          this->logger_,
          info_packet::getErrorState(packet).c_str());
      }
    }
    if (type == info_packet::COMMAND_TYPE::ENCODE_DATA) {
      this->right_encoder_ = info_packet::getRightEncoderData(packet);
      this->left_encode_ = info_packet::getLeftEncoderData(packet);
    }
    if (type == info_packet::COMMAND_TYPE::VOLTAGE) {
      this->voltage_ = info_packet::getVoltage(packet);
    }
  }
}

}  // namespace su065d4380_interface
