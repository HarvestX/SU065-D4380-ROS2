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

#include "su065d4380_tool/param_reader_node.hpp"

namespace su065d4380_tool
{
ParamReaderNode::ParamReaderNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("param_reader_node", options)
{
  this->declare_parameter("dev", "/dev/ttyACM0");
  this->interface_ = std::make_shared<Interface>(
    this->get_parameter("dev").as_string());

  using namespace std::chrono_literals;  // NOLINT
  this->init_timer_ = this->create_wall_timer(500ms, std::bind(&ParamReaderNode::onInit, this));
}

void ParamReaderNode::onInit()
{
  if (this->interface_->on_init() != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize");
    rclcpp::shutdown();
  }

  if (this->interface_->on_configure(State()) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure");
    rclcpp::shutdown();
  }

  if (this->interface_->on_activate(State()) != CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure");
    rclcpp::shutdown();
  }

  this->interface_->readRightGain();
  this->interface_->readLeftGain();
  this->interface_->readAccelerationTime();
  this->interface_->readDecelerationTime();
  this->interface_->readTimeout();
  this->interface_->readInputOffDecelerationTime();

  rclcpp::shutdown();
}
}  // namespace su065d4380_tool

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(su065d4380_tool::ParamReaderNode)
