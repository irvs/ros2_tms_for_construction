#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <regex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "tms_msg_db/srv/tmsdb_get_task.hpp"

// MongoDB関連のインクルード
#include <bsoncxx/json.hpp>
#include <bsoncxx/types.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/uri.hpp>

// leaf nodesのインクルード
#include "tms_ts_primitive/Excavator/leaf_node.hpp"
#include "tms_ts_primitive/Excavator/leaf_node_assist.hpp"
#include "tms_ts_primitive/CrawlerDump/leaf_node.hpp"
#include "tms_ts_primitive/Bulldozer/leaf_node.hpp"
#include "tms_ts_primitive/common/mongo_value_reader.hpp"
#include "tms_ts_primitive/common/blackboard_value_searcher_mongo.hpp"
#include "tms_ts_primitive/common/mongo_value_writer.hpp"
#include "tms_ts_primitive/common/conditional_expression.hpp"
#include "tms_ts_primitive/common/KeepRunningUntilFlgup.hpp"
#include "tms_ts_primitive/common/SetLocalBlackboard.hpp"
#include "tms_ts_primitive/common/SetLocalBlackboardWithCounter.hpp"
#include "tms_ts_primitive/common/Counter.hpp"
#include "tms_ts_primitive/common/wait_for_click.hpp"
//#include "tms_ts_primitive/common/wait_for_topic.hpp"
#include "tms_ts_primitive/common/leaf_node.hpp"
#include "tms_ts_primitive/common/wait_for_ur.hpp"


using namespace BT;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

bool cancelRequested = false;



class TaskSearcher : public rclcpp::Node
{
public:
    TaskSearcher() : Node("task_searcher")
    {
        client_ = this->create_client<tms_msg_db::srv::TmsdbGetTask>("/tms_db_reader_subtask");
    }
    std::optional<std::string> get_task_by_name(const std::string& task_name)
    {
        // task_list_は (task_id, task_sequence の JSON or XML文字列) のペア
        for (const auto& [id, task_str] : task_list_)
        {
            // task_strはJSONならrapidjsonでパースしてtask_nameフィールドを探す例
            rapidjson::Document doc;
            doc.Parse(task_str.c_str());
            if (doc.HasParseError()) continue;
            if (doc.HasMember("task_name") && doc["task_name"].IsString())
            {
                std::string name_in_task = doc["task_name"].GetString();
                if (name_in_task == task_name)
                {
                    // task_sequence文字列を返す（例えば doc["task_sequence"] を文字列化）
                    if (doc.HasMember("task_sequence") && doc["task_sequence"].IsString())
                    {
                        return std::string(doc["task_sequence"].GetString());
                    }
                    else
                    {
                        // task_sequenceフィールドがない場合はtask_strをそのまま返すかnullopt
                        return std::nullopt;
                    }
                }
            }
        }
        return std::nullopt;
    }

    bool is_valid_taskid() const { return is_valid_taskid_; }
    std::string get_task_sequence() const { return task_sequence_; }

    // 実行
    void search_task(const std::string& task_name)
    {
    search_task_impl(task_name);  // 外からはこっちが呼ばれる
    }

private:
    rclcpp::Client<tms_msg_db::srv::TmsdbGetTask>::SharedPtr client_;
    std::vector<std::pair<int64_t, std::string>> task_list_;
    std::string task_sequence_;
    bool is_valid_taskid_ = false;

    void search_task_impl(const std::string& task_name)
    {
        task_list_.clear();
        is_valid_taskid_ = false;

        // クライアントの準備ができるまで待機
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Waiting for service /tms_db_reader_task...");
        }

        auto request = std::make_shared<tms_msg_db::srv::TmsdbGetTask::Request>();
        request->task_name = task_name;

        auto future = client_->async_send_request(request);
        if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
          RCLCPP_ERROR(this->get_logger(), "Service call failed while searching task_name: %s", task_name.c_str());
          return;
        }

        auto response = future.get();
        if (response->task.empty()) {
          RCLCPP_WARN(this->get_logger(), "No tasks found in response.");
          return;
        }

        // 複数タスクの文字列をパース
        std::string tasks_str = response->task;
        // <root>タグを削除
        std::regex action_regex(R"(<root\s+main_tree_to_execute="BehaviorTree">\s*<BehaviorTree\s+ID="BehaviorTree">|</root>)");
        std::string tasks_str_remove_root = std::regex_replace(tasks_str, action_regex, "");
        std::regex after_regex(R"(</BehaviorTree><TreeNodesModel>[\s\S]*)");
        std::string tasks_str_remove_last = std::regex_replace(tasks_str_remove_root, after_regex, "");

        RCLCPP_INFO(this->get_logger(), "Received task: %s", tasks_str_remove_last.c_str());

      task_sequence_ = tasks_str_remove_last;
      is_valid_taskid_ = true;

    }
};



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
    
    factory.registerNodeType<LeafNodeExcavator>("LeafNodeExcavator");
    factory.registerNodeType<LeafNodeExcavatorAssist>("LeafNodeExcavatorAssist");
    factory.registerNodeType<LeafNodeCrawlerDump>("LeafNodeCrawlerDump");
    factory.registerNodeType<LeafNodeBulldozer>("LeafNodeBulldozer");
    factory.registerNodeType<MongoValueReader>("MongoValueReader");
    factory.registerNodeType<BlackboardValueSearcherMongo>("BlackboardValueSearcherMongo");
    factory.registerNodeType<MongoValueWriter>("MongoValueWriter");
    factory.registerNodeType<ConditionalExpression>("ConditionalExpression");
    factory.registerNodeType<KeepRunningUntilFlgup>("KeepRunningUntilFlgup");
    factory.registerNodeType<SetLocalBlackboard>("SetLocalBlackboard");
    factory.registerNodeType<SetLocalBlackboardWithCounter>("SetLocalBlackboardWithCounter");
    factory.registerNodeType<Counter>("Counter");
    factory.registerNodeType<WaitForClick>("WaitForClick");
    //factory.registerNodeType<WaitForTopic>("WaitForTopic");
    factory.registerNodeType<LeafNodeCommon>("LeafNodeCommon");

    // loadBlackboardFromMongoDB("global_blackboard");
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    status_ = NodeStatus::RUNNING;
    
    std::string task_name = this->get_parameter("task_name").as_string();
    int zmq_server_port = this->get_parameter("zmq_server_port").as_int();
    int zmq_publisher_port = this->get_parameter("zmq_publisher_port").as_int();

    int task_id = this->get_parameter("task_id").as_int();
    std::string incoming = std::string(msg->data);

    // --------------------
    // ① JSONが来た場合
    // --------------------
    if (is_json_format(incoming))
    {
      auto result = extract_task_from_json(incoming, task_id);

      if (!result.has_value())
      {
          RCLCPP_ERROR(this->get_logger(),"Task_id %d not found in JSON", task_id);
          return;
      }
      task_sequence_ = result.value();
    }
    // --------------------
    // ② task_name検索モード
    // --------------------
    else if (!task_name.empty() && task_searcher_)
    {
      task_searcher_->search_task(task_name);
      if (!task_searcher_->is_valid_taskid())
      {
        RCLCPP_ERROR(this->get_logger(),"No task found for task_name: %s",task_name.c_str());
        return;
      }
      task_sequence_ = task_searcher_->get_task_sequence();
    }
    // --------------------
    // ③ 直接XML
    // --------------------
    else
    {
      task_sequence_ = incoming;
    }

    // 共通処理
    std::string updated_sequence =replace_whole_execute_subtask_tags(task_sequence_);
    tree_ = factory.createTreeFromText(updated_sequence, bb_);


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
      RCLCPP_ERROR(get_logger(), "Failed to cretae tree: %s", e.what());
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
      case NodeStatus::RUNNING:
      case NodeStatus::IDLE:
        break;
    }

    subscription_.reset();
  }


  bool is_json_format(const std::string& str)
  {
    return !str.empty() && str.front() == '{';
  }

  std::optional<std::string> extract_task_from_json(const std::string& json_str,int target_task_id)
  {
    rapidjson::Document document;
    document.Parse(json_str.c_str());

    if (document.HasParseError())
        return std::nullopt;

    if (!document.HasMember("tasks") || !document["tasks"].IsArray())
        return std::nullopt;

    const rapidjson::Value& tasks = document["tasks"];

    for (rapidjson::SizeType i = 0; i < tasks.Size(); i++)
    {
        const rapidjson::Value& task = tasks[i];

        if (task.HasMember("task_id") &&
            task["task_id"].IsInt() &&
            task["task_id"].GetInt() == target_task_id)
        {
            if (task.HasMember("task_sequence") &&
                task["task_sequence"].IsString())
            {
                return std::string(task["task_sequence"].GetString());
            }
        }
    }
    return std::nullopt;
  }


  std::string process_whole_tag(const std::string& full_tag)
  {
      std::regex attr_regex(R"((\w+)="([^"]*))");
      std::map<std::string, std::string> params;

      auto begin = std::sregex_iterator(full_tag.begin(), full_tag.end(), attr_regex);
      auto end = std::sregex_iterator();

      for (auto it = begin; it != end; ++it) {
          std::string key = (*it)[1];
          std::string value = (*it)[2];
          params[key] = value;
      }

      // subtask_nameがなければそのまま返す
      if (params.find("subtask_name") == params.end())
          return full_tag;

      //  ① subtask_parametersを先に展開
      if (params.find("subtask_parameters") != params.end())
      {
          auto extra_params = parse_subtask_parameters(params["subtask_parameters"]);

          for (const auto& [k, v] : extra_params)
          {
              params[k] = v;
          }
      }

      std::string subtask_name = params["subtask_name"];

      task_searcher_->search_task(subtask_name);

      if (!task_searcher_->is_valid_taskid()) {
          return "<!-- Task not found -->";
      }

      std::string task_sequence = task_searcher_->get_task_sequence();

      //  ② まとめて置換
      for (const auto& [key, value] : params)
      {
          if (key == "ID" || key == "subtask_name" || key == "subtask_parameters")
              continue;

          std::string placeholder = "$" + key + "$";
          replace_all(task_sequence, placeholder, value);
      }

      // 未置換の $変数$ を "none" にする
      task_sequence = replace_unresolved_with_none(task_sequence);

      return task_sequence;
  }

  std::string replace_unresolved_with_none(const std::string& input)
{
    std::regex placeholder_regex(R"(\$([a-zA-Z0-9_]+)\$)");

    auto begin = std::sregex_iterator(input.begin(), input.end(), placeholder_regex);
    auto end = std::sregex_iterator();

    for (auto it = begin; it != end; ++it)
    {
        RCLCPP_WARN(this->get_logger(), "Unresolved param: %s", (*it)[1].str().c_str());
    }

    return std::regex_replace(input, placeholder_regex, "none");
}




  void replace_all(std::string& str, const std::string& from, const std::string& to)
  {
    if (from.empty()) return;

    size_t start_pos = 0;
    while ((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length();
    }
  }

  
  std::map<std::string, std::string> parse_subtask_parameters(const std::string& input)
  {
      std::map<std::string, std::string> result;

      std::stringstream ss(input);
      std::string pair;

      while (std::getline(ss, pair, ','))
      {
          size_t pos = pair.find(':');
          if (pos == std::string::npos) continue;

          std::string key = pair.substr(0, pos);
          std::string value = pair.substr(pos + 1);

          result[key] = value;
      }

      return result;
  }


  std::string replace_whole_execute_subtask_tags(const std::string& xml) {
    std::string modified = xml;

    // タグ全体にマッチ：<Action ID="ExecuteSubtask" ... />
    std::regex action_regex(R"(<Action\s+ID="ExecuteSubtask"[^/>]*/>)");
    std::smatch match;
    std::string::const_iterator searchStart(modified.cbegin());

    while (std::regex_search(searchStart, modified.cend(), match, action_regex)) {
        std::string original_tag = match.str();
        std::string new_tag = process_whole_tag(original_tag);

        // 文字列全体から該当位置を探して置換
        size_t pos = modified.find(original_tag, searchStart - modified.cbegin());
        if (pos != std::string::npos) {
            modified.replace(pos, original_tag.length(), new_tag);
            searchStart = modified.begin() + pos + new_tag.length();
        } else {
            break;
        }
    }

    return modified;
  }

  void set_task_searcher(std::shared_ptr<TaskSearcher> task_searcher)
  {
    task_searcher_ = task_searcher;
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

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  BehaviorTreeFactory factory;
  BT::Tree tree_;
  std::shared_ptr<Blackboard> bb_;
  std::string task_sequence_;
  NodeStatus status_ = NodeStatus::RUNNING;
  std::shared_ptr<TaskSearcher> task_searcher_;  // ← 追加
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
  auto bb = BT::Blackboard::create();

  auto task_searcher = std::make_shared<TaskSearcher>();
  auto exec_task_sequence = std::make_shared<ExecTaskSequence>(bb);

  exec_task_sequence->declare_parameter<std::string>("task_name", "");

  exec_task_sequence->set_task_searcher(task_searcher);  // ← 接続！

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(task_searcher);
  exec.add_node(exec_task_sequence);

  auto force_quiet_node = std::make_shared<ForceQuietNode>();
  exec.add_node(force_quiet_node);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}