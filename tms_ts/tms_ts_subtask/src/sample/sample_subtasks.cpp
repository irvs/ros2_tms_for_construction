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
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "tms_ts_subtask/subtask_base.hpp"
#include "tms_ts_subtask/sample/sample_subtasks.hpp"

using namespace BT;
using namespace std::chrono_literals;

Sample::Sample(const std::string& name, const NodeConfiguration& config) : BaseClassSubtasks("sample_subtask", config){
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/zx120/boom/cmd",10);
  }

PortsList Sample::providedPorts() { return { InputPort<float>("initial_position")}; }
NodeStatus Sample::tick(){
  Optional<float> initial_position = getInput<float>("initial_position");
  Optional<float> goal_position = getInput<float>("goal_position");
  int param_num;

  while(true){
    param_num = GetParamFromDB("sample", "parameter_value");
    RCLCPP_INFO_STREAM(node_->get_logger(),"param_num: " << param_num << "  from sample_subtask");
    sleep(1);
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Complete sample subtask");
  return NodeStatus::SUCCESS;
}



