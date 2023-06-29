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
SU065D4380Interface::SU065D4380Interface(const std::string & dev)
{
  this->port_handler_ = std::make_unique<PortHandler>(dev);
}

SU065D4380Interface::~SU065D4380Interface()
{
  this->port_handler_.reset();
}

CallbackReturn SU065D4380Interface::on_configure(const State &)
{
  if (!this->port_handler_->configure(115200)) {
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SU065D4380Interface::on_activate(const State &)
{
  if (!this->port_handler_->open()) {
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SU065D4380Interface::on_deactivate(const State &)
{
  // TODO(anyone): Stop motor;

  if (!this->port_handler_->close()) {
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

void SU065D4380Interface::read() noexcept
{
  std::stringstream buf;
  this->port_handler_->readUntil(buf, '\r');

  return;
}

void SU065D4380Interface::write() noexcept
{

}


const rclcpp::Logger SU065D4380Interface::getLogger() noexcept
{
  return rclcpp::get_logger("SU065D4380Interface");
}

}  // namespace su065d4380_interface
