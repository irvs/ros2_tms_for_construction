#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include "tms_msg_db/srv/tmsdb_get_task.hpp" 
#include "tms_msg_ts/action/simple_connector_hfsmbth.hpp"

#include "tms_ts_subtask/Excavator/leaf_node.hpp"
#include "tms_ts_subtask/CrawlerDump/leaf_node.hpp"
#include "tms_ts_subtask/Bulldozer/leaf_node.hpp"

#include "tms_ts_subtask/common/blackboard_value_reader_mongo.hpp"
#include "tms_ts_subtask/common/mongo_value_writer.hpp"
#include "tms_ts_subtask/common/conditional_expression.hpp"
#include "tms_ts_subtask/common/KeepRunningUntilFlgup.hpp"
#include "tms_ts_subtask/common/SetLocalBlackboard.hpp"
#include "tms_ts_subtask/common/SetLocalBlackboardWithCounter.hpp"
#include "tms_ts_subtask/common/Counter.hpp"
#include "tms_ts_subtask/common/wait_for_click.hpp"

using TmsdbGetTask = tms_msg_db::srv::TmsdbGetTask;
using SimpleConnectorHFSMBTH = tms_msg_ts::action::SimpleConnectorHFSMBTH;
using GoalHandleSimpleConnectorHFSMBTH = rclcpp_action::ServerGoalHandle<SimpleConnectorHFSMBTH>;
using namespace std::chrono_literals;

class ExecTaskActionServer : public rclcpp::Node
{
public:
  ExecTaskActionServer()
  : Node("exec_task_action_server"),
    zmq_server_port_(declare_parameter<int>("zmq_server_port", 1666)),
    zmq_publisher_port_(declare_parameter<int>("zmq_publisher_port", 1777)),
    tick_hz_(declare_parameter<double>("tick_hz", 20.0)),
    srv_task_name_(declare_parameter<std::string>("srv_task_name", "/tms_db_reader_task"))

  {
    // BT ノード登録
    factory_.registerNodeType<LeafNodeExcavator>("LeafNodeExcavator");
    factory_.registerNodeType<LeafNodeCrawlerDump>("LeafNodeCrawlerDump");
    factory_.registerNodeType<LeafNodeBulldozer>("LeafNodeBulldozer");
    factory_.registerNodeType<BlackboardValueReaderMongo>("BlackboardValueReaderMongo");
    factory_.registerNodeType<MongoValueWriter>("MongoValueWriter");
    factory_.registerNodeType<ConditionalExpression>("ConditionalExpression");
    factory_.registerNodeType<KeepRunningUntilFlgup>("KeepRunningUntilFlgup");
    factory_.registerNodeType<SetLocalBlackboard>("SetLocalBlackboard");
    factory_.registerNodeType<SetLocalBlackboardWithCounter>("SetLocalBlackboardWithCounter");
    factory_.registerNodeType<Counter>("Counter");
    factory_.registerNodeType<WaitForClick>("WaitForClick");

    bb_ = BT::Blackboard::create();

    cli_task_ = this->create_client<TmsdbGetTask>(srv_task_name_);

    using std::placeholders::_1;
    using std::placeholders::_2;

    action_server_ = rclcpp_action::create_server<SimpleConnectorHFSMBTH>(
      this, "SimpleConnectorHFSMBTH",
      std::bind(&ExecTaskActionServer::on_goal, this, _1, _2),
      std::bind(&ExecTaskActionServer::on_cancel, this, _1),
      std::bind(&ExecTaskActionServer::on_accepted, this, _1)
    );

  }

private:

  rclcpp_action::GoalResponse on_goal(const rclcpp_action::GoalUUID&,
                                      std::shared_ptr<const SimpleConnectorHFSMBTH::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Goal received: task_id=%d", goal->task_id);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }



  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandleSimpleConnectorHFSMBTH>)
  {
    RCLCPP_WARN(get_logger(), "Cancel request -> Emergency halt");
    cancel_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  

  void on_accepted(const std::shared_ptr<GoalHandleSimpleConnectorHFSMBTH> gh)
  {
    std::thread(&ExecTaskActionServer::do_execute, this, gh).detach();
  }


  void do_execute(const std::shared_ptr<GoalHandleSimpleConnectorHFSMBTH> gh)
  {
    auto goal = gh->get_goal();
    cancel_.store(false);

    // Blackboard に Goal 情報を流しておく（BT 内から参照できるように）
    // bb_->set<int>("task_id", goal->task_id);
    // bb_->set<std::string>("model_name", goal->model_name);
    // bb_->set<std::string>("parameter_name", goal->parameter_name);

    // 1) DB (Service) から BT XML 取得
    std::string bt_xml;
    if (!fetch_bt_xml(goal->task_id, bt_xml))
    {
      auto r = std::make_shared<SimpleConnectorHFSMBTH::Result>();
      r->result_code = 1; // RESULT_FAILED
      RCLCPP_INFO(get_logger(), "Failed to fetch BT XML from DB");
      gh->abort(r);
      return;
    }

    // 2) BT 構築
    BT::Tree tree;
    try {
      tree = factory_.createTreeFromText(bt_xml, bb_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "BT creation error: %s", e.what());
      auto r = std::make_shared<SimpleConnectorHFSMBTH::Result>();
      r->result_code = 1; // RESULT_FAILED
      RCLCPP_INFO(get_logger(), (std::string("BT creation error: ") + e.what()).c_str());
      gh->abort(r);
      return;
    }

    // Groot 可視化（任意）
    BT::PublisherZMQ pub(tree, 100, zmq_server_port_, zmq_publisher_port_);

    // 3) 実行ループ（feedback 送信 & cancel 対応）
    auto fb = std::make_shared<SimpleConnectorHFSMBTH::Feedback>();
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    rclcpp::Rate rate(tick_hz_);
    int ticks = 0;

    while (rclcpp::ok() && status == BT::NodeStatus::RUNNING)
    {
      if (cancel_.load() || gh->is_canceling())
      {
        tree.rootNode()->halt();
        auto r = std::make_shared<SimpleConnectorHFSMBTH::Result>();
        r->result_code = 2; // RESULT_ABORTED
        RCLCPP_INFO(get_logger(), "Emergency stop");
        gh->canceled(r);
        return;
      }

      status = tree.tickRoot();
      ++ticks;

      RCLCPP_INFO(get_logger(), "Node execution status is: %s",
                (status == BT::NodeStatus::RUNNING) ? "RUNNING" :
                (status == BT::NodeStatus::SUCCESS) ? "SUCCESS" : "FAILURE");
      gh->publish_feedback(fb);

      rate.sleep();
    }

    // 4) 結果
    auto r = std::make_shared<SimpleConnectorHFSMBTH::Result>();
    if (status == BT::NodeStatus::SUCCESS) {
      r->result_code = 0; // RESULT_OK
      RCLCPP_INFO(get_logger(), "Task finished successfully");
      gh->succeed(r);
    } else {
      r->result_code = 1; // RESULT_FAILED
      RCLCPP_INFO(get_logger(), "Task failed");
      gh->abort(r);
    }
  }

  bool fetch_bt_xml(int32_t task_id, std::string& out_xml)
  {
    if (cli_task_->wait_for_service(0s))
    {
      auto req = std::make_shared<TmsdbGetTask::Request>();
      req->task_id = task_id;

      auto fut = cli_task_->async_send_request(req);
      while (rclcpp::ok() && fut.wait_for(50ms) != std::future_status::ready) { /* wait */ }

      if (fut.valid())
      {
        auto res = fut.get();
        if (res && !res->task.empty()) {
          out_xml = res->task;
          RCLCPP_INFO(get_logger(), "Fetched BT from TmsdbGetTask (task_id=%d)", task_id);
          return true;
        }
      }
      RCLCPP_WARN(get_logger(), "TmsdbGetTask returned empty / invalid for task_id=%d", task_id);
    }

    // // 次に汎用 TmsdbGetData を試す（あなたが提示したリクエスト項目に合わせる）
    // if (cli_data_->wait_for_service(0s))
    // {
    //   auto req = std::make_shared<tms_msg_db::srv::TmsdbGetData::Request>();
    //   req->type        = model_name;       // 例：モデル種別で絞る（必要に応じて変更）
    //   req->id          = task_id;          // 例：task_id で絞る
    //   req->latest_only = true;
    //   req->flgorparam  = parameter_name;   // 例：追加フィルタに利用
    //   req->name        = model_name;       // 例：nameにも同値を入れておく
    //   // req->recordnames = {...}            // 使わないなら空のまま

    //   auto fut = cli_data_->async_send_request(req);
    //   while (rclcpp::ok() && fut.wait_for(50ms) != std::future_status::ready) { /* wait */ }

    //   if (fut.valid())
    //   {
    //     auto res = fut.get();
    //     if (res && !res->tmsdbs.empty())
    //     {
    //       // ★ どのフィールドに BT XML を入れているかは環境依存
    //       //   典型: Tmsdb の string 系フィールド（task / note / etcdata 等）を順に見る
    //       for (const auto& rec : res->tmsdbs)
    //       {
    //         // 以下は例。あなたの Tmsdb メッセージに合わせて差し替えてください。
    //         if (!rec.task.empty()) { out_xml = rec.task; return true; }
    //         if (!rec.note.empty()) { out_xml = rec.note; return true; }
    //         if (!rec.etcdata.empty()) { out_xml = rec.etcdata; return true; }
    //       }
    //       RCLCPP_WARN(get_logger(), "TmsdbGetData found records but no BT XML field matched");
    //     }
    //     else {
    //       RCLCPP_WARN(get_logger(), "TmsdbGetData returned no records for task_id=%d", task_id);
    //     }
    //   }
    // }

    return false;
  }

private:
  // Action
  rclcpp_action::Server<SimpleConnectorHFSMBTH>::SharedPtr action_server_;
  std::atomic_bool cancel_{false};

  // BT
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr bb_;

  // DB services
  rclcpp::Client<TmsdbGetTask>::SharedPtr cli_task_;
  std::string srv_task_name_;
  //   std::string srv_data_name_;

  // Params
  int    zmq_server_port_;
  int    zmq_publisher_port_;
  double tick_hz_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<ExecTaskActionServer>();
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}