#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

// MongoDB関連のインクルード
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>

// leaf nodesのインクルード
#include "tms_ts_subtask/OPERA/sample/zx120/sample_leaf_nodes.hpp"
#include "tms_ts_subtask/OPERA/sample/zx200/sample_leaf_nodes.hpp"
#include "tms_ts_subtask/FUJITA/mst2200/leaf_node.hpp"
#include "tms_ts_subtask/OPERA/ic120/leaf_node.hpp"
#include "tms_ts_subtask/OPERA/zx200/leaf_node.hpp"
#include "tms_ts_subtask/OPERA/mst110cr/leaf_node.hpp"
#include "tms_ts_subtask/common/blackboard_value_checker.hpp"
#include "tms_ts_subtask/common/blackboard_value_writer_topic.hpp"
#include "tms_ts_subtask/common/blackboard_value_writer_srv.hpp"
#include "tms_ts_subtask/common/blackboard_value_reader_mongo.hpp"
#include "tms_ts_subtask/common/mongo_value_writer.hpp"
#include "tms_ts_subtask/common/conditional_expression.hpp"
#include "tms_ts_subtask/common/conditional_expression_bool.hpp"
// #include "tms_ts_subtask/zx200/excavation_area_segmenter.hpp"
#include "tms_ts_subtask/common/KeepRunningUntilFlgup.hpp"
#include "tms_ts_subtask/common/SetLocalBlackboard.hpp"

using namespace BT;
using namespace std::chrono_literals;

bool cancelRequested = false;

class ExecTaskSequence : public rclcpp::Node
{
public:
  ExecTaskSequence(std::shared_ptr<Blackboard> bb) : Node("exec_task_sequence"), bb_(bb)
  {
    this->declare_parameter("task_id", -1);
    this->declare_parameter("zmq_server_port", 1666);
    this->declare_parameter("zmq_publisher_port", 1777);
    
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/task_sequence", 10, std::bind(&ExecTaskSequence::topic_callback, this, std::placeholders::_1));
    
    factory.registerNodeType<LeafNodeMst2200>("LeafNodeMst2200");
    factory.registerNodeType<LeafNodeIc120>("LeafNodeIc120");
    factory.registerNodeType<LeafNodeMst110cr>("LeafNodeMst110cr");
    factory.registerNodeType<LeafNodeSampleZx120>("LeafNodeSampleZx120");
    factory.registerNodeType<LeafNodeSampleZx200>("LeafNodeSampleZx200");
    factory.registerNodeType<LeafNodeZx200>("LeafNodeZx200");
    factory.registerNodeType<BlackboardValueChecker>("BlackboardValueChecker");
    factory.registerNodeType<BlackboardValueWriterTopic>("BlackboardValueWriterTopic");
    factory.registerNodeType<BlackboardValueWriterSrv>("BlackboardValueWriterSrv");
    factory.registerNodeType<BlackboardValueReaderMongo>("BlackboardValueReaderMongo");
    factory.registerNodeType<MongoValueWriter>("MongoValueWriter");
    factory.registerNodeType<ConditionalExpression>("ConditionalExpression");
    factory.registerNodeType<ConditionalExpressionBool>("ConditionalExpressionBool");
    // factory.registerNodeType<ExcavationAreaSegmenter>("ExcavationAreaSegmenter");
    factory.registerNodeType<KeepRunningUntilFlgup>("KeepRunningUntilFlgup");
    factory.registerNodeType<SetLocalBlackboard>("SetLocalBlackboard");

    // loadBlackboardFromMongoDB("SAMPLE_BLACKBOARD_SIMIZU");
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    int task_id = this->get_parameter("task_id").as_int();
    int zmq_server_port = this->get_parameter("zmq_server_port").as_int();
    int zmq_publisher_port = this->get_parameter("zmq_publisher_port").as_int();
    
    if (task_id == -1) {
      // 既存の処理：そのままBehavior Treeとして実行
      task_sequence_ = std::string(msg->data);
      tree_ = factory.createTreeFromText(task_sequence_, bb_);
    } else {
      // 新しい処理：JSONデータから特定のtask_idのタスクを抽出
      std::string json_data = std::string(msg->data);
      
      rapidjson::Document document;
      document.Parse(json_data.c_str());
      
      if (document.HasParseError()) {
        RCLCPP_ERROR(this->get_logger(), "JSON parse error");
        return;
      }
      
      if (!document.HasMember("tasks") || !document["tasks"].IsArray()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid JSON format: missing 'tasks' array");
        return;
      }
      
      const rapidjson::Value& tasks = document["tasks"];
      bool task_found = false;
      
      for (rapidjson::SizeType i = 0; i < tasks.Size(); i++) {
        const rapidjson::Value& task = tasks[i];
        
        if (task.HasMember("task_id") && task["task_id"].IsInt() &&
            task.HasMember("task_sequence") && task["task_sequence"].IsString()) {
          
          if (task["task_id"].GetInt() == task_id) {
            task_sequence_ = task["task_sequence"].GetString();
            tree_ = factory.createTreeFromText(task_sequence_, bb_);
            task_found = true;
            RCLCPP_INFO(this->get_logger(), "Found and executing task_id: %d", task_id);
            break;
          }
        }
      }
      
      if (!task_found) {
        RCLCPP_ERROR(this->get_logger(), "Task with task_id %d not found in JSON data", task_id);
        return;
      }
    }

    BT::PublisherZMQ publisher_zmq(tree_, 100, zmq_server_port, zmq_publisher_port);
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
  void loadBlackboardFromMongoDB(const std::string& record_name)
  {
    // mongocxx::instance instance{};
    mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
    mongocxx::database db = client["rostmsdb"];
    mongocxx::collection collection = db["parameter"];

    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "record_name" << record_name;
    auto filter = filter_builder.view();
    auto doc = collection.find_one(filter);

    if (doc)
    {
      auto view = doc->view();
      for (auto&& element : view)
      {
        std::string key = element.key().to_string();
        auto value = element.get_value();

        if (key != "_id" && key != "model_name" && key != "type" && key != "record_name") {

          switch (value.type())
          {
            case bsoncxx::type::k_utf8:
              bb_->set(key, value.get_utf8().value.to_string());
              break;
            case bsoncxx::type::k_int32:
              bb_->set(key, value.get_int32().value);
              break;
            case bsoncxx::type::k_int64:
              bb_->set(key, value.get_int64().value);
              break;
            case bsoncxx::type::k_double:
              bb_->set(key, value.get_double().value);
              break;
            case bsoncxx::type::k_bool:
              bb_->set(key, value.get_bool().value);
              break;
            default:
              std::cerr << "Unsupported BSON type: " << bsoncxx::to_string(value.type()) << std::endl;
              break;
          }
        }
      }
    bb_->set("CHECK_TRUE", true);
    bb_->set("CHECK_FALSE", false);
    bb_->set("TERMINATE_FLG", false);
    bb_->set("STANDBY_FLG", false);
    }
    else
    {
      std::cerr << "Couldn't find document with record_name: " << record_name << std::endl;
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
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