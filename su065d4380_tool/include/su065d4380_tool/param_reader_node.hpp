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

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <su065d4380_interface/param_reader_interface.hpp>
#include <su065d4380_parameters.hpp>

namespace su065d4380_tool
{
class ParamReaderNode : public rclcpp::Node
{
private:
  using CallbackReturn = su065d4380_interface::CallbackReturn;
  using Interface = su065d4380_interface::ParamReaderInterface;
  using State = su065d4380_interface::State;

  rclcpp::TimerBase::SharedPtr init_timer_;
  Interface::SharedPtr interface_;
  std::unique_ptr<su065d4380_param::ParamListener> param_listener_;

public:
  explicit ParamReaderNode(const rclcpp::NodeOptions &);

private:
  void onInit();
};
}  // namespace su065d4380_tool
