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

using namespace BT;
using namespace std::chrono_literals;

class SubtaskControlZx120Boom : public SyncActionNode {
public:
  SubtaskControlZx120Boom(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg, radian;
};


class SubtaskControlZx120Swing : public SyncActionNode {
public:
  SubtaskControlZx120Swing(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg, radian;
};


class SubtaskControlZx120Arm : public SyncActionNode {
public:
  SubtaskControlZx120Arm(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg, radian;
};



class SubtaskControlZx120Bucket : public SyncActionNode {
public:
  SubtaskControlZx120Bucket(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();
  NodeStatus tick() override;
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg, radian;
};

#endif