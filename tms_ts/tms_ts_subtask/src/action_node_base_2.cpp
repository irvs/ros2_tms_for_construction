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

#include "tms_ts_subtask/action_node_base_2.hpp"

using std::placeholders::_1;
using namespace BT;
using namespace std::chrono_literals;


BaseClassSubtasks::BaseClassSubtasks(const std::string& name, const NodeConfiguration& config) : CoroActionNode(name, config){
    node_ = rclcpp::Node::make_shared(name);
    DatabaseManager::getInstance();
    
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive,
      false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    goal_ = tms_msg_ts::action::ActionSample::Goal();
    goal_.goal_pos = -40; //action serverに送るgoalの値をここで設定
    result_ = rclcpp_action::ClientGoalHandle<tms_msg_ts::action::ActionSample>::WrappedResult();
    BaseClassSubtasks::createActionClient(action_name_);
}

// action clientを新たに作成する関数
void BaseClassSubtasks::createActionClient(const std::string & action_name){
    action_client_ = rclcpp_action::create_client<tms_msg_ts::action::ActionSample>(node_, action_name, callback_group_);

    RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", action_name.c_str());
    if (!action_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(
        node_->get_logger(), "\"%s\" action server not available after waiting for 5 s",
        action_name.c_str());
      throw std::runtime_error(
              std::string("Action server ") + action_name +
              std::string(" not available"));
    } 
}

// action clientのgoal,feedback,resultに対応するcallback関数をこの関数内で定義
void BaseClassSubtasks::send_new_goal(){
    goal_result_available_ = false;
    auto send_goal_options = typename rclcpp_action::Client<tms_msg_ts::action::ActionSample>::SendGoalOptions();
    
    // actionのresultに対応するcallback関数をここで定義
    send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::ActionSample>::WrappedResult & result) {
        if (this->goal_handle_->get_goal_id() == result.goal_id) {
          goal_result_available_ = true;
          result_ = result;
        }
      };
    // actionのfeedbackに対応するcallback関数をここで定義
    send_goal_options.feedback_callback =
      [this](typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::ActionSample>::SharedPtr,
        const std::shared_ptr<const typename tms_msg_ts::action::ActionSample::Feedback> feedback) {
        feedback_ = feedback;
      };

    //actionのgoalに対応するcallback関数をここで定義
    future_goal_handle_ = std::make_shared<
      std::shared_future<typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::ActionSample>::SharedPtr>>(
      action_client_->async_send_goal(goal_, send_goal_options));

    // time_goal_sent_変数にactionのgoalを送信した時刻を格納
    time_goal_sent_ = node_->now();
}


  bool BaseClassSubtasks::is_future_goal_handle_complete(std::chrono::milliseconds & elapsed)
  {
    auto remaining = server_timeout_ - elapsed;
    if (remaining <= std::chrono::milliseconds(0)) {
      future_goal_handle_.reset();
      return false;
    }
    auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;
    auto result =
      callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
    elapsed += timeout;
    if (result == rclcpp::FutureReturnCode::INTERRUPTED) {
      future_goal_handle_.reset();
      throw std::runtime_error("send_goal failed");
    }
    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      goal_handle_ = future_goal_handle_->get();
      future_goal_handle_.reset();
      if (!goal_handle_) {
        throw std::runtime_error("Goal was rejected by the action server");
      }
      return true;
    }
    return false;
  }


NodeStatus BaseClassSubtasks::tick() {
  // tick関数が呼び出された際にstatus()==NodeStatus::IDLEの場合はNodeStatus::RUNNINGに変更して、action serverへgoalを送信する
  if (status() == NodeStatus::IDLE) {
    setStatus(NodeStatus::RUNNING);
    should_send_goal_ = true;
    BaseClassSubtasks::on_tick();
    if (!should_send_goal_) {
        return NodeStatus::FAILURE;
      }
    BaseClassSubtasks::send_new_goal();
  }

  //action serverからの通信を受け取るまでの時間を計測し、server_timeout_を超えた場合にtick関数はNodeStatus::FAILUREを返す
  try {
    if (future_goal_handle_) {
      auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
      if (!is_future_goal_handle_complete(elapsed)) {
        if (elapsed < server_timeout_) {
          return BT::NodeStatus::RUNNING;
        }
        RCLCPP_WARN(
          node_->get_logger(),
          "Timed out while waiting for action server to acknowledge goal request for %s",
          action_name_.c_str());
        future_goal_handle_.reset();
        return BT::NodeStatus::FAILURE;
      }
    }

    // action serverへgoalを送ってからresultを受け取るまでの間にif文の中が実行される
    if (rclcpp::ok() && !goal_result_available_) {
      on_wait_for_result(feedback_);　//この関数実行時にブロッキングが起きるため、長い処理はこの関数内で実行しない
      feedback_.reset();
      auto goal_status = goal_handle_->get_status();
      if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
        goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
      {
        goal_updated_ = false;
        send_new_goal();
        auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
        if (!is_future_goal_handle_complete(elapsed)) {
          if (elapsed < server_timeout_) {
            return BT::NodeStatus::RUNNING;
          }
          RCLCPP_WARN(
            node_->get_logger(),
            "Timed out while waiting for action server to acknowledge goal request for %s",
            action_name_.c_str());
          future_goal_handle_.reset();
          return BT::NodeStatus::FAILURE;
        }
      }

      //callback関数を1回だけ実行する
      callback_group_executor_.spin_some();

      //action_serverからresultが返ってきていなかったら、tick関数はNodeStatus::RUNNINGを返す
      if (!goal_result_available_) {
        return BT::NodeStatus::RUNNING;
      }
    }
  } catch (const std::runtime_error & e) {
    if (e.what() == std::string("send_goal failed") ||
      e.what() == std::string("Goal was rejected by the action server"))
    {
      return BT::NodeStatus::FAILURE;
    } else {
      throw e;
    }
  }
  
  // サブタスク側のaction serverからactionのresultを受け取って、NodeStatusに変換しtick()の戻り値とする部分
  switch (result_.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Succeeded the Subtask");
      bt_status = NodeStatus::SUCCESS;
      break;

    case rclcpp_action::ResultCode::ABORTED:
      bt_status = NodeStatus::FAILURE;
      break;

    case rclcpp_action::ResultCode::CANCELED:
    RCLCPP_INFO(node_->get_logger(), "Canceled the Subtask");
      bt_status = NodeStatus::FAILURE;
      break;

    default:
      throw std::logic_error("BtActionNode::Tick: invalid status value");
  }

  goal_handle_.reset();
  return bt_status;
};


// データベースから動的パラメータの値をとってくるための関数
std::map<std::string, int> BaseClassSubtasks::GetParamFromDB(int parameter_id){
    //std::lock_guard<std::mutex> lock(db_instance_mutex);
    mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
    mongocxx::database db = client["rostmsdb"];
    mongocxx::collection collection = db["parameter"];
    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "parameter_id" << parameter_id;
    auto filter = filter_builder.view();
    auto result = collection.find_one(filter);
    if (result) {
        std::map<std::string, int> dataMap;

        for (auto&& element : result->view()) {
            std::string key = element.key().to_string();
            if (key != "_id" && key != "type" && key != "description" && key != "parameter_id" && key != "parameter_type" && key != "parts_name") {
                if (element.type() == bsoncxx::type::k_int32) {
                    int value = element.get_int32();
                    dataMap[key] = value;
                }
            }
        }

        return dataMap;
    } else {
        std::cout << "Dynamic parameter not found in your parameter collection" << std::endl;
        return std::map<std::string, int>();
    }
}