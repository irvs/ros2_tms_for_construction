// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BASE_CLASS_SUBTASKS_HPP
#define BASE_CLASS_SUBTASKS_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

using std::placeholders::_1;
using namespace BT;
using namespace std::chrono_literals;

class BaseClassSubtasks : public CoroActionNode{
public:
    BaseClassSubtasks(const std::string& name, const NodeConfiguration& config);
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Node::SharedPtr node_;
    // NodeStatus shutdown_node(const std_msgs::msg::String & msg) const;
};

#endif