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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "tms_ts_subtask/subtask_base.hpp"

using namespace BT;
using namespace std::chrono_literals;

SubtaskControlZx120Boom::SubtaskControlZx120Boom(const std::string& name, const NodeConfiguration& config) : BaseClassSubtasks("subtask_zx120_boom_sample", config){
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/zx120/boom/cmd",10);
  }
PortsList SubtaskControlZx120Boom::providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
NodeStatus SubtaskControlZx120Boom::tick(){
  Optional<float> initial_position = getInput<float>("initial_position");
  Optional<float> goal_position = getInput<float>("goal_position");
  RCLCPP_INFO_STREAM(node_->get_logger(),"Boom joint : initial_position: " << initial_position.value() << "   "<< "goal_position: " << goal_position.value());
  if (!initial_position){ throw RuntimeError("missing required input [initial_position]: ", initial_position.error() ); }
  if (!initial_position){ throw RuntimeError("missing required input [goal_position]: ", goal_position.error() ); }
  deg = initial_position.value();
  while(deg >= goal_position.value()){
      deg += goal_position.value() / float(20.0);
      msg_rad.data = float(deg * float(M_PI / 180));
      publisher_->publish(msg_rad);
      RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing boom position: " << deg << " [deg]");
      sleep(1);
      setStatusRunningAndYield();
      // mco_yield(NodeStatus::RUNNING);
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Complete boom modification");
  return NodeStatus::SUCCESS;
}

