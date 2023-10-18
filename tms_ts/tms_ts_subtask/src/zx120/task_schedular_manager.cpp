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