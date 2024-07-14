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
#include "std_msgs/msg/bool.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

// sample leaf nodes
#include "tms_ts_subtask/sample/zx120/sample_leaf_nodes.hpp"
#include "tms_ts_subtask/sample/zx200/sample_leaf_nodes.hpp"

// leaf nodes for ic120
#include "tms_ts_subtask/ic120/leaf_node.hpp"
// leaf nodes for zx200
#include "tms_ts_subtask/zx200/leaf_node.hpp"
// commmon action nodes
#include "tms_ts_subtask/common/blackboard_value_checker.hpp"
#include "tms_ts_subtask/common/blackboard_value_writer_topic.hpp"
#include "tms_ts_subtask/common/blackboard_value_writer_srv.hpp"




using namespace BT;
using namespace std::chrono_literals;

bool cancelRequested = false;

class ExecTaskSequence : public rclcpp::Node
{
public:
  ExecTaskSequence() : Node("exec_task_sequence")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/task_sequence", 10, std::bind(&ExecTaskSequence::topic_callback, this, std::placeholders::_1));
    
    // Add leafNodes to registrated nodes

    // ic120
    factory.registerNodeType<LeafNodeIc120>("LeafNodeIc120");
    // zx120
    factory.registerNodeType<LeafNodeSampleZx120>("LeafNodeSampleZx120");
    // zx200
    factory.registerNodeType<LeafNodeSampleZx200>("LeafNodeSampleZx200");
    factory.registerNodeType<LeafNodeZx200>("LeafNodeZx200");
    // common nodes
    factory.registerNodeType<BlackboardValueChecker>("BlackboardValueChecker");
    factory.registerNodeType<BlackboardValueWriterTopic>("BlackboardValueWriterTopic");
    factory.registerNodeType<BlackboardValueWriterSrv>("BlackboardValueWriterSrv");


  }
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    task_sequence = std::string(msg->data);
    tree = factory.createTreeFromText(task_sequence);

    // Setup black board parameters
    auto blackboard = tree.rootBlackboard();
    blackboard->set("RepeatSwitching", 0); // Blackboard parameters for Repeat Switching Node


    BT::PublisherZMQ publisher_zmq(tree);
    try
    {
      while (rclcpp::ok() && status == NodeStatus::RUNNING)
      {
        status = tree.tickRoot();
        if (cancelRequested == true)
        {
          tree.rootNode()->halt();
          status = NodeStatus::FAILURE;
        }
      };
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("exec_task_sequence"), "Behavior tree threw an exception");
      status = NodeStatus::FAILURE;
    }

    switch (status)
    {
      case NodeStatus::SUCCESS:
        RCLCPP_INFO_STREAM(rclcpp::get_logger("exec_task_sequence"), "Task is succesfully finished.");
        break;

      case NodeStatus::FAILURE:
        RCLCPP_INFO_STREAM(rclcpp::get_logger("exec_task_sequence"), "Task is canceled.");
        break;
    }

    subscription_.reset();
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  BehaviorTreeFactory factory;
  BT::Tree tree;
  std::string task_sequence;
  NodeStatus status = NodeStatus::RUNNING;
};

class ForceQuietNode : public rclcpp::Node
{
public:
  ForceQuietNode() : Node("force_quiet_node")
  {
    shutdown_subscription = this->create_subscription<std_msgs::msg::Bool>(
        "/emergency_signal", 10, std::bind(&ForceQuietNode::callback, this, _1));
  }

  void callback(const std_msgs::msg::Bool& msg)
  {
    if (msg.data == true)
    {
      cancelRequested = true;
      shutdown_subscription.reset();
    };
  };

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_subscription;
};

int main(int argc, char** argv)
{
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