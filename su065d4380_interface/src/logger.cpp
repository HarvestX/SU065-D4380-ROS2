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

#include "su065d4380_interface/logger.hpp"

namespace su065d4380_interface
{
rclcpp::Logger LoggingInterface::get_logger() const
{
  return rclcpp::get_logger(this->get_logger_name());
}

const char * LoggingInterface::get_logger_name() const
{
  return "SU065D4380Interface";
}
}  // namespace su065d4380_interface
