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

#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <su065d4380_interface/su065d4380_interface.hpp>

namespace su065d4380_tool
{
class KeyboardReader final
{
private:
  struct termios cooked_;

public:
  KeyboardReader();
  ~KeyboardReader();
  char readOne();
};

class ParamConfigurator : public rclcpp::Node
{
public:
  enum class CONFIGURABLE_PARAM
  {
    RIGHT_WHEEL_GAIN,
    LEFT_WHEEL_GAIN,
    ACC_TIME,
    DEC_TIME,
    TIMEOUT,
    DEC_WITH_TIMEOUT,
    END
  };

private:
  const std::string dev_;
  std::shared_ptr<su065d4380_interface::PortHandler> port_handler_;
  std::shared_ptr<su065d4380_interface::ParameterCommander> commander_;

public:
  explicit ParamConfigurator(const rclcpp::NodeOptions &);
  ~ParamConfigurator();

  void readAll() const;
  void read(const CONFIGURABLE_PARAM &) const;
  void write(const CONFIGURABLE_PARAM &, const int) const;

private:
  static const std::string getParamName(const CONFIGURABLE_PARAM &);
};
}  // namespace su065d4380_tool
