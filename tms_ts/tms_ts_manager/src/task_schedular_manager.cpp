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

// leaf nodesÇÃÉCÉìÉNÉãÅ[Éh
#include "tms_ts_subtask/sample/zx120/sample_leaf_nodes.hpp"
#include "tms_ts_subtask/sample/zx200/sample_leaf_nodes.hpp"
#include "tms_ts_subtask/ic120/leaf_node.hpp"
#include "tms_ts_subtask/zx200/leaf_node.hpp"
#include "tms_ts_subtask/common/blackboard_value_checker.hpp"
#include "tms_ts_subtask/common/blackboard_value_writer_topic.hpp"
#include "tms_ts_subtask/common/blackboard_value_writer_srv.hpp"
#include "tms_ts_subtask/common/blackboard_value_reader_mongo.hpp"
#include "tms_ts_subtask/common/mongo_value_writer.hpp"
#include "tms_ts_subtask/common/conditional_expression.hpp"

using namespace BT;
using namespace std::chrono_literals;

bool cancelRequested = false;

class ExecTaskSequence : public rclcpp::Node
{
public:
  ExecTaskSequence(std::shared_ptr<Blackboard> global_bb) : Node("exec_task_sequence"), global_bb_(global_bb)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/task_sequence", 10, std::bind(&ExecTaskSequence::topic_callback, this, std::placeholders::_1));
    
    // ÉmÅ[ÉhÇìoò^
    factory.registerNodeType<LeafNodeIc120>("LeafNodeIc120");
    factory.registerNodeType<LeafNodeSampleZx120>("LeafNodeSampleZx120");
    factory.registerNodeType<LeafNodeSampleZx200>("LeafNodeSampleZx200");
    factory.registerNodeType<LeafNodeZx200>("LeafNodeZx200");
    factory.registerNodeType<BlackboardValueChecker>("BlackboardValueChecker");
    factory.registerNodeType<BlackboardValueWriterTopic>("BlackboardValueWriterTopic");
    factory.registerNodeType<BlackboardValueWriterSrv>("BlackboardValueWriterSrv");
    factory.registerNodeType<BlackboardValueReaderMongo>("BlackboardValueReaderMongo");
    factory.registerNodeType<MongoValueWriter>("MongoValueWriter");
    factory.registerNodeType<ConditionalExpression>("ConditionalExpression");

    bb_ = Blackboard::create(global_bb_);
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    task_sequence_ = std::string(msg->data);
    tree_ = factory.createTreeFromText(task_sequence_, bb_);

    BT::PublisherZMQ publisher_zmq(tree_, 100, 1666, 1777);
    try
    {
      while (rclcpp::ok() && status_ == NodeStatus::RUNNING)
      {
        status_ = tree_.tickRoot();
        if (cancelRequested == true)
        {
          tree_.rootNode()->halt();
          status_ = NodeStatus::FAILURE;
        }
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("exec_task_sequence"), "Behavior tree threw an exception");
      status_ = NodeStatus::FAILURE;
    }

    switch (status_)
    {
      case NodeStatus::SUCCESS:
        RCLCPP_INFO_STREAM(rclcpp::get_logger("exec_task_sequence"), "Task is successfully finished.");
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
  BT::Tree tree_;
  std::shared_ptr<Blackboard> bb_;
  std::shared_ptr<Blackboard> global_bb_;
  std::string task_sequence_;
  NodeStatus status_ = NodeStatus::RUNNING;
};

class ForceQuietNode : public rclcpp::Node
{
public:
  ForceQuietNode() : Node("force_quiet_node")
  {
    shutdown_subscription = this->create_subscription<std_msgs::msg::Bool>(
        "/emergency_signal", 10, std::bind(&ForceQuietNode::callback, this, std::placeholders::_1));
  }

  void callback(const std_msgs::msg::Bool& msg)
  {
    if (msg.data == true)
    {
      cancelRequested = true;
      shutdown_subscription.reset();
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr shutdown_subscription;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto global_bb = Blackboard::create();
  global_bb->set("sample_param_global", 0);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto task_schedular_node = std::make_shared<ExecTaskSequence>(global_bb);
  exec.add_node(task_schedular_node);
  auto force_quiet_node = std::make_shared<ForceQuietNode>();
  exec.add_node(force_quiet_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
