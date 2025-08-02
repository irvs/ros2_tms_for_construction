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
#include "tms_msg_ur/msg/demo202412.hpp"

// MongoDB�֘A�̃C���N���[�h
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>

// leaf nodes�̃C���N���[�h
#include "tms_ts_subtask/FUJITA/mst2200/leaf_node.hpp"
#include "tms_ts_subtask/OPERA/d37pxi/leaf_node.hpp"
#include "tms_ts_subtask/OPERA/ic120/leaf_node.hpp"
#include "tms_ts_subtask/OPERA/zx200/leaf_node.hpp"
#include "tms_ts_subtask/OPERA/mst110cr/leaf_node.hpp"
#include "tms_ts_subtask/common/blackboard_value_reader_mongo.hpp"
#include "tms_ts_subtask/common/mongo_value_writer.hpp"
#include "tms_ts_subtask/common/conditional_expression.hpp"
#include "tms_ts_subtask/common/KeepRunningUntilFlgup.hpp"
#include "tms_ts_subtask/common/SetLocalBlackboard.hpp"
#include "tms_ts_subtask/common/SetLocalBlackboardWithCounter.hpp"
#include "tms_ts_subtask/common/Counter.hpp"

using namespace BT;
using namespace std::chrono_literals;

bool cancelRequested = false;

class ExecTaskSequence : public rclcpp::Node
{
public:
  ExecTaskSequence(std::shared_ptr<Blackboard> bb) : Node("task_schedular_manager1_202412"), bb_(bb)
  {
    subscription_ = this->create_subscription<tms_msg_ur::msg::Demo202412>(
        "/task_sequence", 10, std::bind(&ExecTaskSequence::topic_callback, this, std::placeholders::_1));
    
    factory.registerNodeType<LeafNodeD37pxi>("LeafNodeD37pxi");
    factory.registerNodeType<LeafNodeMst2200>("LeafNodeMst2200");
    factory.registerNodeType<LeafNodeIc120>("LeafNodeIc120");
    factory.registerNodeType<LeafNodeMst110cr>("LeafNodeMst110cr");
    factory.registerNodeType<LeafNodeZx200>("LeafNodeZx200");
    factory.registerNodeType<BlackboardValueReaderMongo>("BlackboardValueReaderMongo");
    factory.registerNodeType<MongoValueWriter>("MongoValueWriter");
    factory.registerNodeType<ConditionalExpression>("ConditionalExpression");
    factory.registerNodeType<KeepRunningUntilFlgup>("KeepRunningUntilFlgup");
    factory.registerNodeType<SetLocalBlackboard>("SetLocalBlackboard");
    factory.registerNodeType<SetLocalBlackboardWithCounter>("SetLocalBlackboardWithCounter");
    factory.registerNodeType<Counter>("Counter");

    // loadBlackboardFromMongoDB("SAMPLE_BLACKBOARD_SHIMIZU");
  }

  void topic_callback(const tms_msg_ur::msg::Demo202412::SharedPtr msg)
  {
    task_sequence_ = std::string(msg->task_sequence1);
       // tree_ = factory.createTreeFromText(task_sequence_, bb_);

    try {
      tree_ = factory.createTreeFromText(task_sequence_, bb_);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(),
        "Failed to cretae tree: %s", e.what());
      subscription_.reset();
      return;
    }


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
      RCLCPP_ERROR(rclcpp::get_logger("exec_task_sequence"), "Behavior tree threw an exception: %s", e.what());
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
  // void loadBlackboardFromMongoDB(const std::string& record_name)
  // {
  //   // mongocxx::instance instance{};
  //   mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
  //   mongocxx::database db = client["rostmsdb"];
  //   mongocxx::collection collection = db["parameter"];

  //   bsoncxx::builder::stream::document filter_builder;
  //   filter_builder << "record_name" << record_name;
  //   auto filter = filter_builder.view();
  //   auto doc = collection.find_one(filter);

  //   if (doc)
  //   {
  //     auto view = doc->view();
  //     for (auto&& element : view)
  //     {
  //       std::string key = element.key().to_string();
  //       auto value = element.get_value();

  //       if (key != "_id" && key != "model_name" && key != "type" && key != "record_name") {

  //         switch (value.type())
  //         {
  //           case bsoncxx::type::k_utf8:
  //             bb_->set(key, value.get_utf8().value.to_string());
  //             break;
  //           case bsoncxx::type::k_int32:
  //             bb_->set(key, value.get_int32().value);
  //             break;
  //           case bsoncxx::type::k_int64:
  //             bb_->set(key, value.get_int64().value);
  //             break;
  //           case bsoncxx::type::k_double:
  //             bb_->set(key, value.get_double().value);
  //             break;
  //           case bsoncxx::type::k_bool:
  //             bb_->set(key, value.get_bool().value);
  //             break;
  //           default:
  //             std::cerr << "Unsupported BSON type: " << bsoncxx::to_string(value.type()) << std::endl;
  //             break;
  //         }
  //       }
  //     }
  //   bb_->set("CHECK_TRUE", true);
  //   bb_->set("CHECK_FALSE", false);
  //   bb_->set("TERMINATE_FLG", false);
  //   bb_->set("STANDBY_FLG", false);
  //   }
  //   else
  //   {
  //     std::cerr << "Couldn't find document with record_name: " << record_name << std::endl;
  //   }
  // }

  rclcpp::Subscription<tms_msg_ur::msg::Demo202412>::SharedPtr subscription_;
  BehaviorTreeFactory factory;
  BT::Tree tree_;
  std::shared_ptr<Blackboard> bb_;
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
  auto bb = Blackboard::create();

  rclcpp::executors::MultiThreadedExecutor exec;
  auto task_schedular_node = std::make_shared<ExecTaskSequence>(bb);
  exec.add_node(task_schedular_node);
  auto force_quiet_node = std::make_shared<ForceQuietNode>();
  exec.add_node(force_quiet_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
