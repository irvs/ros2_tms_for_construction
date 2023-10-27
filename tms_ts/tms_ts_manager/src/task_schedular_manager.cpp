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

#include "tms_ts_subtask/zx120/subtask_base_parts.hpp"
#include "tms_ts_subtask/zx120/subtasks.hpp"
#include "tms_ts_subtask/zx200/subtask_base_parts.hpp"
#include "tms_ts_subtask/zx200/subtasks.hpp"

using namespace BT;
using namespace std::chrono_literals;

class ExecTaskSequence : public rclcpp::Node{
public:
  ExecTaskSequence() : Node("exec_task_sequence"){
    std::string task_sequence;
    subscription_ = this->create_subscription<std_msgs::msg::String>("/task_sequence", 10, std::bind(&ExecTaskSequence::topic_callback, this, std::placeholders::_1));
    // zx120
    factory.registerNodeType<SubtaskControlZx120Boom>("SubtaskControlZx120Boom");
    factory.registerNodeType<SubtaskControlZx120Swing>("SubtaskControlZx120Swing");
    factory.registerNodeType<SubtaskControlZx120Arm>("SubtaskControlZx120Arm");
    factory.registerNodeType<SubtaskControlZx120Bucket>("SubtaskControlZx120Bucket");
    // zx200
    factory.registerNodeType<SubtaskControlZx200Boom>("SubtaskControlZx200Boom");
    factory.registerNodeType<SubtaskControlZx200Swing>("SubtaskControlZx200Swing");
    factory.registerNodeType<SubtaskControlZx200Arm>("SubtaskControlZx200Arm");
    factory.registerNodeType<SubtaskControlZx200Bucket>("SubtaskControlZx200Bucket");
    
  }
  void topic_callback(const std_msgs::msg::String::SharedPtr msg){
    task_sequence = std::string(msg->data);
    tree = factory.createTreeFromText(task_sequence);
    BT::PublisherZMQ publisher_zmq(tree);
    NodeStatus status = NodeStatus::RUNNING;
    while(rclcpp::ok() && status == NodeStatus::RUNNING){ 
        status = tree.tickRoot();
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "status: " << status);
    }
  }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    BehaviorTreeFactory factory;
    BT::Tree tree;
    std::string task_sequence;
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExecTaskSequence>());
  rclcpp::shutdown();
  return 0;
}