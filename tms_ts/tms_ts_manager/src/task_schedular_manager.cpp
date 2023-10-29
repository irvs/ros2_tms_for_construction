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
#include "tms_ts_subtask/zx120/sample_subtasks.hpp"
#include "tms_ts_subtask/zx200/sample_subtasks.hpp"

using namespace BT;
using namespace std::chrono_literals;

bool shutdown_requested = false;

class ExecTaskSequence : public rclcpp::Node{
public:
  ExecTaskSequence() : Node("exec_task_sequence"){
    subscription_ = this->create_subscription<std_msgs::msg::String>("/task_sequence", 10, std::bind(&ExecTaskSequence::topic_callback, this, std::placeholders::_1));
    
    // ic120
    
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
    
    std::thread behavior_tree_thread([this] {
      NodeStatus status = tree.tickRoot();
      RCLCPP_INFO_STREAM(rclcpp::get_logger("exec_task_sequence"), "status: " << status);
    });
    behavior_tree_thread.detach();

    while(rclcpp::ok() && status == NodeStatus::RUNNING){ 
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if(!rclcpp::ok() || shutdown_requested){
          exit(0);
        }
    }
  }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_, shutdown_subscription;
    BehaviorTreeFactory factory;
    BT::Tree tree;
    std::string task_sequence;
    NodeStatus status = NodeStatus::RUNNING;
};

class ForceQuietNode : public rclcpp::Node{
public:
  ForceQuietNode() : Node("force_quiet_node"){
    shutdown_subscription = this->create_subscription<std_msgs::msg::String>("/emergency_signal", 10,std::bind(&ForceQuietNode::sample_callback, this, _1));
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr shutdown_subscription;
  void sample_callback(const std_msgs::msg::String & msg) const{
    if (msg.data == "EMERGENCY") {
        shutdown_requested = true;
  };
  };
};

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto task_schedular_node = std::make_shared<ExecTaskSequence>();
  exec.add_node(task_schedular_node);
  auto force_quiet_node = std::make_shared<ForceQuietNode>();
  exec.add_node(force_quiet_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}