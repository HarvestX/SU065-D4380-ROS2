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

#include "su065d4380_interface/su065d4380_interface.hpp"

namespace su065d4380_interface
{
CallbackReturn SU065D4380Interface::on_init()
{
  this->rx_left_vel_packet_ = std::make_unique<RxLeftVelPacket>();
  this->rx_right_vel_packet_ = std::make_unique<RxRightVelPacket>();
  this->rx_drv_packet_ = std::make_unique<RxDrvPacket>();
  this->rx_enc_packet_ = std::make_unique<RxEncPacket>();
  this->rx_vol_packet_ = std::make_unique<RxVolPacket>();
  this->tx_rx_vel_packet_ = std::make_unique<TxRxVelPacket>();

  return CallbackReturn::SUCCESS;
}

CallbackReturn SU065D4380Interface::on_activate(const State &)
{
  const auto & ret = this->port_handler_->open();

  // Need to set velocity for initial communication
  if (ret) {
    this->setVelocity(0.0, 0.0);
    this->write();
  }
  return ret ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

SU065D4380Interface::~SU065D4380Interface()
{
  this->rx_left_vel_packet_.reset();
  this->rx_right_vel_packet_.reset();
  this->rx_drv_packet_.reset();
  this->rx_enc_packet_.reset();
  this->rx_vol_packet_.reset();
  this->tx_rx_vel_packet_.reset();
}

void SU065D4380Interface::readSinglePacket(const std::string & packet)
{
  if (packet.size() < 1 || packet.at(0) != '$') {
    return;
  }

  if (packet.size() == RxInfoPacketBase::ASCII_BUF_SIZE &&
    packet[RxInfoPacketBase::ID_IDX] == RxInfoPacketBase::ID)
  {
    switch (packet[RxInfoPacketBase::SUB_ID_IDX]) {
      case RxLeftVelPacket::SUB_ID:
        this->rx_left_vel_packet_->set(packet);
        break;
      case RxRightVelPacket::SUB_ID:
        this->rx_right_vel_packet_->set(packet);
        break;
      case RxDrvPacket::SUB_ID:
        this->rx_drv_packet_->set(packet);
        break;
      case RxEncPacket::SUB_ID:
        this->rx_enc_packet_->set(packet);
        break;
      case RxVolPacket::SUB_ID:
        this->rx_vol_packet_->set(packet);
        break;
      default:
        RCLCPP_WARN(
          this->getLogger(), "Invalid packet ID given %c",
          packet[RxInfoPacketBase::SUB_ID_IDX]);
        break;
    }
  } else if (packet.size() == RxVelPacket::ASCII_BUF_SIZE) {
    try {
      this->tx_rx_vel_packet_->setRx(packet);
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR(this->getLogger(), e.what());
    }
  }
}

void SU065D4380Interface::write() noexcept
{
  std::string buf;
  if (!this->tx_rx_vel_packet_->getTx(buf)) {
    RCLCPP_WARN(this->getLogger(), "Waiting response from driver...");
    return;
  }
  this->port_handler_->write(buf.data(), buf.size());
}

void SU065D4380Interface::consumeAll() noexcept
{
  this->rx_left_vel_packet_->consume();
  this->rx_right_vel_packet_->consume();
  this->rx_drv_packet_->consume();
  this->rx_vol_packet_->consume();
}

bool SU065D4380Interface::hasError() noexcept
{
  if (!this->rx_drv_packet_->isOK()) {
    return false;
  }

  const error_state_t e = this->rx_drv_packet_->getErrorState();

  return e != error_state_t::OK;
}

void SU065D4380Interface::showError() noexcept
{
  if (!this->rx_drv_packet_->isOK()) {
    return;
  }

  const error_state_t e = this->rx_drv_packet_->getErrorState();
  if (e == error_state_t::OK) {
    RCLCPP_INFO(this->getLogger(), RxDrvPacket::getErrorStr(e).c_str());
  } else {
    RCLCPP_ERROR(this->getLogger(), RxDrvPacket::getErrorStr(e).c_str());
  }
}

bool SU065D4380Interface::hasSolvableError() noexcept
{
  if (!this->rx_drv_packet_->isOK()) {
    return false;
  }

  const error_state_t e = this->rx_drv_packet_->getErrorState();
  return RxDrvPacket::isSolvableError(e);
}

void SU065D4380Interface::solveError() noexcept
{
  this->tx_rx_vel_packet_->setVelocity(
    mode_flag_t::FLAG_MODE_ERROR_REST, 0, 0);
  this->write();
}

double SU065D4380Interface::getRightVelocity() noexcept
{
  if (!this->rx_right_vel_packet_->isOK()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return this->rx_right_vel_packet_->getRPM() * RPM2RPS;
}

double SU065D4380Interface::getLeftVelocity() noexcept
{
  if (!this->rx_left_vel_packet_->isOK()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return this->rx_left_vel_packet_->getRPM() * RPM2RPS;
}

double SU065D4380Interface::getRightRadian() noexcept
{
  if (!this->rx_enc_packet_->isOK()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return this->rx_enc_packet_->getRightEncoder() * ENC2RAD;
}

double SU065D4380Interface::getLeftRadian() noexcept
{
  if (!this->rx_enc_packet_->isOK()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  return this->rx_enc_packet_->getLeftEncoder() * ENC2RAD;
}

void SU065D4380Interface::setVelocity(const double left_rps, const double right_rps) noexcept
{
  this->tx_rx_vel_packet_->setVelocity(
    mode_flag_t::FLAG_MODE_MOTOR_ON,
    static_cast<int16_t>(left_rps * RPS2RPM), static_cast<int16_t>(-1.0 * right_rps * RPS2RPM));
}

const rclcpp::Logger SU065D4380Interface::getLogger() noexcept
{
  return rclcpp::get_logger("SU065D4380Interface");
}
}  // namespace su065d4380_interface
