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

#include "zx120/subtasks.hpp"

using namespace BT;
using namespace std::chrono_literals;

SubtaskControlZx120Boom::SubtaskControlZx120Boom(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){
    node_ = rclcpp::Node::make_shared("subtask_zx120_boom_sample");
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
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Complete boom modification");
  return NodeStatus::SUCCESS;
}


SubtaskControlZx120Swing::SubtaskControlZx120Swing(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){
    node_ = rclcpp::Node::make_shared("subtask_zx120_swing_sample");
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/zx120/swing/cmd",10);
}
PortsList SubtaskControlZx120Swing::providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
  NodeStatus SubtaskControlZx120Swing::tick(){
    Optional<float> initial_position = getInput<float>("initial_position"); // max:continuous 
    Optional<float> goal_position = getInput<float>("goal_position"); // min:continuous
    RCLCPP_INFO_STREAM(node_->get_logger(),"Swing joint : initial_position: " << initial_position.value() << "   "<< "goal_position: " << goal_position.value());
    if (!initial_position){ throw RuntimeError("missing required input [initial_position]: ", initial_position.error() ); }
    if (!initial_position){ throw RuntimeError("missing required input [goal_position]: ", goal_position.error() ); }
    deg = initial_position.value();
    while(deg <= goal_position.value()){
        deg += goal_position.value() / float(20.0);
        msg_rad.data = float(deg * float(M_PI / 180));
        publisher_->publish(msg_rad);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing swing position: " << deg << " [deg]");
        sleep(1);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Complete swing modification");
    return NodeStatus::SUCCESS;
}


SubtaskControlZx120Arm::SubtaskControlZx120Arm(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){
    node_ = rclcpp::Node::make_shared("subtask_zx120_arm_sample");
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/zx120/arm/cmd",10);
}
PortsList SubtaskControlZx120Arm::providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
NodeStatus SubtaskControlZx120Arm::tick(){
  Optional<float> initial_position = getInput<float>("initial_position");  // min:30[deg]
  Optional<float> goal_position = getInput<float>("goal_position"); // max:152[deg]
  RCLCPP_INFO_STREAM(node_->get_logger(),"Arm joint : initial_position: " << initial_position.value() << "   "<< "goal_position: " << goal_position.value());
  if (!initial_position){ throw RuntimeError("missing required input [initial_position]: ", initial_position.error() ); }
  if (!initial_position){ throw RuntimeError("missing required input [goal_position]: ", goal_position.error() ); }
  deg = initial_position.value();
  while(deg >= goal_position.value()){
      deg -= goal_position.value() / float(20.0);
      msg_rad.data = float(deg * float(M_PI / 180));
      publisher_->publish(msg_rad);
      RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing arm position: " << deg << " [deg]");
      sleep(1);
  }
  RCLCPP_INFO_STREAM(node_->get_logger(), "Complete arm modification");
  return NodeStatus::SUCCESS;
}


SubtaskControlZx120Bucket::SubtaskControlZx120Bucket(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){
    node_ = rclcpp::Node::make_shared("subtask_zx120_bucket_sample");
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/zx120/bucket/cmd",10);
  }
PortsList SubtaskControlZx120Bucket::providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
NodeStatus SubtaskControlZx120Bucket::tick(){
    Optional<float> initial_position = getInput<float>("initial_position"); // min:-33[deg]
    Optional<float> goal_position = getInput<float>("goal_position"); // max:143[deg]
    RCLCPP_INFO_STREAM(node_->get_logger(),"Bucket joint : initial_position: " << initial_position.value() << "   "<< "goal_position: " << goal_position.value());
    if (!initial_position){ throw RuntimeError("missing required input [initial_position]: ", initial_position.error() ); }
    if (!initial_position){ throw RuntimeError("missing required input [goal_position]: ", goal_position.error() ); }
    deg = initial_position.value();
    while(deg >= goal_position.value()){
        deg -= goal_position.value() / float(20.0);
        msg_rad.data = float(deg * float(M_PI / 180));
        publisher_->publish(msg_rad);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing bucket position: " << deg << " [deg]");
        sleep(1);
    }
    RCLCPP_INFO_STREAM(node_->get_logger(), "Complete bucket modification");
    return NodeStatus::SUCCESS;
  }



