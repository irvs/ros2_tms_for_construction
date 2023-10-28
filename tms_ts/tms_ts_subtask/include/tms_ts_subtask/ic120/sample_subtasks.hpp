// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
 
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
 
//      http://www.apache.org/licenses/LICENSE-2.0
 
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ZX120_SUBTASKS_HPP_
#define ZX120_SUBTASKS_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "tms_ts_subtask/subtask_base.hpp"

using namespace BT;
using namespace std::chrono_literals;

class SubtaskControlZx120Boom : public BaseClassSubtasks {
public:
  SubtaskControlZx120Boom(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  NodeStatus tick() override;
private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg, radian;
};


class SubtaskControlZx120Swing : public BaseClassSubtasks {
public:
  SubtaskControlZx120Swing(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  NodeStatus tick() override;
private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg, radian;
};


class SubtaskControlZx120Arm : public BaseClassSubtasks {
public:
  SubtaskControlZx120Arm(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  NodeStatus tick() override;
private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg, radian;
};



class SubtaskControlZx120Bucket : public BaseClassSubtasks {
public:
  SubtaskControlZx120Bucket(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  NodeStatus tick() override;
private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg, radian;
};

#endif