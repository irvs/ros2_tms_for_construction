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

#include "tms_ts_subtask/leaf_node_base.hpp"

using std::placeholders::_1;
using namespace BT;
using namespace std::chrono_literals;

LeafNodeBase::LeafNodeBase(const std::string& name, const NodeConfiguration& config) : CoroActionNode(name, config)
{
  node_ = rclcpp::Node::make_shared(name);
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  Optional<std::string> model_name = getInput<std::string>("model_name");
  Optional<std::string> record_name = getInput<std::string>("record_name");
  Optional<std::string> subtask_name = getInput<std::string>("subtask_name");
  goal_.model_name = model_name.value();
  goal_.record_name = record_name.value();
  subtask_name_ = subtask_name.value();
  RCLCPP_INFO(node_->get_logger(), "model_name: %s", goal_.model_name.c_str());
  RCLCPP_INFO(node_->get_logger(), "record_name: %s", goal_.record_name.c_str());
  result_ = rclcpp_action::ClientGoalHandle<tms_msg_ts::action::LeafNodeBase>::WrappedResult();
  LeafNodeBase::createActionClient(subtask_name_);
}

// action clientを新たに作成する関数
void LeafNodeBase::createActionClient(const std::string& subtask_name)
{
  action_client_ = rclcpp_action::create_client<tms_msg_ts::action::LeafNodeBase>(node_, subtask_name, callback_group_);

  RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", subtask_name.c_str());
  if (!action_client_->wait_for_action_server(5s))
  {
    RCLCPP_ERROR(node_->get_logger(), "\"%s\" action server not available after waiting for 5 s", subtask_name.c_str());
    throw std::runtime_error(std::string("Action server ") + subtask_name + std::string(" not available"));
  }
}

// action clientのgoal,feedback,resultに対応するcallback関数をこの関数内で定義
void LeafNodeBase::send_new_goal()
{
  goal_result_available_ = false;
  auto send_goal_options = typename rclcpp_action::Client<tms_msg_ts::action::LeafNodeBase>::SendGoalOptions();

  // actionのresultに対応するcallback関数をここで定義
  send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::LeafNodeBase>::WrappedResult& result) {
        if (this->goal_handle_->get_goal_id() == result.goal_id)
        {
          goal_result_available_ = true;
          result_ = result;
        }
      };
  // actionのfeedbackに対応するcallback関数をここで定義
  send_goal_options.feedback_callback =
      [this](typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::LeafNodeBase>::SharedPtr,
             const std::shared_ptr<const typename tms_msg_ts::action::LeafNodeBase::Feedback> feedback) {
        feedback_ = feedback;
      };

  // actionのgoalに対応するcallback関数をここで定義
  future_goal_handle_ = std::make_shared<
      std::shared_future<typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::LeafNodeBase>::SharedPtr>>(
      action_client_->async_send_goal(goal_, send_goal_options));

  // time_goal_sent_変数にactionのgoalを送信した時刻を格納
  time_goal_sent_ = node_->now();
}

bool LeafNodeBase::is_future_goal_handle_complete(std::chrono::milliseconds& elapsed)
{
  auto remaining = server_timeout_ - elapsed;
  if (remaining <= std::chrono::milliseconds(0))
  {
    future_goal_handle_.reset();
    return false;
  }
  auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;
  auto result = callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
  elapsed += timeout;
  if (result == rclcpp::FutureReturnCode::INTERRUPTED)
  {
    future_goal_handle_.reset();
    throw std::runtime_error("send_goal failed");
  }
  if (result == rclcpp::FutureReturnCode::SUCCESS)
  {
    goal_handle_ = future_goal_handle_->get();
    future_goal_handle_.reset();
    if (!goal_handle_)
    {
      throw std::runtime_error("Goal was rejected by the action server");
    }
    return true;
  }
  return false;
}

void LeafNodeBase::halt()
{
  if (should_cancel_goal())
  {
    auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
    if (callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to cancel subtask for %s", subtask_name_.c_str());
    }
  }

  setStatus(BT::NodeStatus::IDLE);
}

bool LeafNodeBase::should_cancel_goal()
{
  if (status() != BT::NodeStatus::RUNNING)
  {
    return false;
  }
  if (!goal_handle_)
  {
    return false;
  }

  callback_group_executor_.spin_some();
  auto status = goal_handle_->get_status();

  return status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
         status == action_msgs::msg::GoalStatus::STATUS_EXECUTING;
}

NodeStatus LeafNodeBase::tick()
{
  // tick関数が呼び出された際にstatus()==NodeStatus::IDLEの場合はNodeStatus::RUNNINGに変更して、action
  // serverへgoalを送信する
  if (status() == NodeStatus::IDLE)
  {
    setStatus(NodeStatus::RUNNING);
    should_send_goal_ = true;
    if (!should_send_goal_)
    {
      return NodeStatus::FAILURE;
    }
    LeafNodeBase::send_new_goal();
  }

  // action
  // serverからの通信を受け取るまでの時間を計測し、server_timeout_を超えた場合にtick関数はNodeStatus::FAILUREを返す
  try
  {
    if (future_goal_handle_)
    {
      auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
      if (!is_future_goal_handle_complete(elapsed))
      {
        if (elapsed < server_timeout_)
        {
          return BT::NodeStatus::RUNNING;
        }
        RCLCPP_WARN(node_->get_logger(), "Timed out while waiting for subtask to acknowledge goal request for %s",
                    subtask_name_.c_str());
        future_goal_handle_.reset();
        return BT::NodeStatus::FAILURE;
      }
    }

    // action serverへgoalを送ってからresultを受け取るまでの間にif文の中が実行される
    if (rclcpp::ok() && !goal_result_available_)
    {
      feedback_.reset();
      auto goal_status = goal_handle_->get_status();
      if (goal_updated_ && (goal_status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
                            goal_status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED))
      {
        goal_updated_ = false;
        send_new_goal();
        auto elapsed = (node_->now() - time_goal_sent_).to_chrono<std::chrono::milliseconds>();
        if (!is_future_goal_handle_complete(elapsed))
        {
          if (elapsed < server_timeout_)
          {
            return BT::NodeStatus::RUNNING;
          }
          RCLCPP_WARN(node_->get_logger(),
                      "Timed out while waiting for action server to acknowledge goal request for %s subatsk",
                      subtask_name_.c_str());
          future_goal_handle_.reset();
          return BT::NodeStatus::FAILURE;
        }
      }

      // callback関数を1回だけ実行する
      callback_group_executor_.spin_some();

      // action_serverからresultが返ってきていなかったら、tick関数はNodeStatus::RUNNINGを返す
      if (!goal_result_available_)
      {
        return BT::NodeStatus::RUNNING;
      }
    }
  }
  catch (const std::runtime_error& e)
  {
    if (e.what() == std::string("send_goal failed") || e.what() == std::string("Goal was rejected by the action "
                                                                               "server"))
    {
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      throw e;
    }
  }

  // サブタスク側のaction serverからactionのresultを受け取って、NodeStatusに変換しtick()の戻り値とする部分
  switch (result_.code)
  {
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
