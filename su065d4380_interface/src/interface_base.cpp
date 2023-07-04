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

#include "su065d4380_interface/interface_base.hpp"

namespace su065d4380_interface
{
InterfaceBase::InterfaceBase(const std::string & dev)
{
  this->port_handler_ = std::make_unique<PortHandler>(dev);
}

InterfaceBase::~InterfaceBase()
{
  this->port_handler_.reset();
}

CallbackReturn InterfaceBase::on_configure(const State &)
{
  return this->port_handler_->configure(115200) ?
         CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

CallbackReturn InterfaceBase::on_activate(const State &)
{
  return this->port_handler_->open() ?
         CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

CallbackReturn InterfaceBase::on_deactivate(const State &)
{
  return this->port_handler_->close() ?
         CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

void InterfaceBase::read()
{
  std::stringstream buf;
  this->port_handler_->readUntil(buf, '\r');
  std::string packet;
  while (std::getline(buf, packet, '\r')) {
    this->readSinglePacket(packet);
  }
}
}  // namespace su065d4380_interface
