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
PacketHandler::PacketHandler(
  PortHandlerBase const * const port_handler
)
: port_handler_(port_handler),
  pool_(std::make_unique<PacketPool>())
{

}

int PacketHandler::writePort(
  char const * const packet, const int length) const
{
  return this->port_handler_->writePort(packet, length);
}

int PacketHandler::readPortIntoQueue()
{
  char buf[100];
  int ret = this->port_handler_->readPort(buf, sizeof(buf));
  this->pool_->enqueue(std::string(buf));
  return ret;
}

int PacketHandler::getBytesAvailable() const
{
  return this->port_handler_->getBytesAvailable();
}

bool PacketHandler::takeVelocityPacket(std::string & out)
{
  return this->pool_->takeVelocityPacket(out);
}

bool PacketHandler::takeInfoPacket(std::string & out)
{
  return this->pool_->takeInfoPacket(out);
}

bool PacketHandler::takeParamPacket(std::string & out)
{
  return this->pool_->takeParamPacket(out);
}
}  // namespace su065d4380_interface
