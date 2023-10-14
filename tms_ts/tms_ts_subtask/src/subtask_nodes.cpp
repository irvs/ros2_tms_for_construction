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

using namespace BT;
using namespace std::chrono_literals;


class SubtaskControlZx120Boom : public SyncActionNode{
// boom joint (degrees : -70 to 44)
// pitch角は値が小さくなるほどzx120の上方向に持ち上がる
// 初期姿勢で関節角を20度以上にしてしまうとバケットが地面に付き、zx120自体を持ち上げてしまうので注意
public:
  SubtaskControlZx120Boom(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){
    node_ = rclcpp::Node::make_shared("subtask_zx120_boom_sample");
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/zx120/boom/cmd",10);
  }
  static PortsList providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
  NodeStatus tick() override{
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
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg=0, radian=0;
};


class SubtaskControlZx120Swing : public SyncActionNode{
public:
// swing joint (degrees: continuous)
  SubtaskControlZx120Swing(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){
    node_ = rclcpp::Node::make_shared("subtask_zx120_swing_sample");
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/zx120/swing/cmd",10);
  }
  static PortsList providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
  NodeStatus tick() override{
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
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg=0, radian=0;
};


class SubtaskControlZx120Arm : public SyncActionNode{
// arm joint (degrees : 30 to 152)
public:
  SubtaskControlZx120Arm(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){
    node_ = rclcpp::Node::make_shared("subtask_zx120_arm_sample");
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/zx120/arm/cmd",10);
  }
  static PortsList providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
  NodeStatus tick() override{
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
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg=0, radian=0;
};


class SubtaskControlZx120Bucket : public SyncActionNode{
//bucket joint (degrees : -33 to 143)
public:
  SubtaskControlZx120Bucket(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config){
    node_ = rclcpp::Node::make_shared("subtask_zx120_bucket_sample");
    publisher_ = node_->create_publisher<std_msgs::msg::Float64>("/zx120/bucket/cmd",10);
  }
  static PortsList providedPorts() { return { InputPort<float>("initial_position"), InputPort<float>("goal_position") }; }
  NodeStatus tick() override{
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
private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
  std_msgs::msg::Float64 msg_initial_position, msg_goal_position, msg_rad;
  float deg=0, radian=0;
};

class ExecTaskSequence : public rclcpp::Node{
public:
  ExecTaskSequence() : Node("exec_task_sequence"){
    std::string task_sequence;
    subscription_ = this->create_subscription<std_msgs::msg::String>("/task_sequence", 10, std::bind(&ExecTaskSequence::topic_callback, this, std::placeholders::_1));
    factory.registerNodeType<SubtaskControlZx120Boom>("SubtaskControlZx120Boom");
    factory.registerNodeType<SubtaskControlZx120Swing>("SubtaskControlZx120Swing");
    factory.registerNodeType<SubtaskControlZx120Arm>("SubtaskControlZx120Arm");
    factory.registerNodeType<SubtaskControlZx120Bucket>("SubtaskControlZx120Bucket");
    
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