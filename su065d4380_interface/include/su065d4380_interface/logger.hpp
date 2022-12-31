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

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace su065d4380_interface
{
class LoggingInterface : public rclcpp::node_interfaces::NodeLoggingInterface
{
public:
  using SharedPtr = std::shared_ptr<LoggingInterface>;

public:
  rclcpp::Logger get_logger() const override;
  const char * get_logger_name() const override;
};
}  // namespace su065d4380_interface
