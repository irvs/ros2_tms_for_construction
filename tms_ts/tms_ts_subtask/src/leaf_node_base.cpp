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

// action clientを新たに作�?�する関数
void LeafNodeBase::createActionClient(const std::string& subtask_name)
{
  auto options_client = rcl_action_client_get_default_options();
  options_client.goal_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_client.result_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_client.cancel_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_client.feedback_topic_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_client.status_topic_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();

  action_client_ = rclcpp_action::create_client<tms_msg_ts::action::LeafNodeBase>(node_, subtask_name, callback_group_, options_client);

  RCLCPP_DEBUG(node_->get_logger(), "Waiting for \"%s\" action server", subtask_name.c_str());
  if (!action_client_->wait_for_action_server(5s))
  {
    RCLCPP_ERROR(node_->get_logger(), "\"%s\" action server not available after waiting for 5 s", subtask_name.c_str());
    throw std::runtime_error(std::string("Action server ") + subtask_name + std::string(" not available"));
  }
}

// action clientのgoal,feedback,resultに対応するcallback関数をこの関数�?で定義
void LeafNodeBase::send_new_goal()
{
  goal_result_available_ = false;
  auto send_goal_options = typename rclcpp_action::Client<tms_msg_ts::action::LeafNodeBase>::SendGoalOptions();

  send_goal_options.result_callback =
      [this](const typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::LeafNodeBase>::WrappedResult& result) {
        if (this->goal_handle_->get_goal_id() == result.goal_id)
        {
          goal_result_available_ = true;
          result_ = result;
        }
      };

  send_goal_options.feedback_callback =
      [this](typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::LeafNodeBase>::SharedPtr,
             const std::shared_ptr<const typename tms_msg_ts::action::LeafNodeBase::Feedback> feedback) {
        feedback_ = feedback;
      };

  future_goal_handle_ = std::make_shared<
      std::shared_future<typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::LeafNodeBase>::SharedPtr>>(
      action_client_->async_send_goal(goal_, send_goal_options));

  time_goal_sent_ = node_->now();
}



bool LeafNodeBase::is_future_goal_handle_complete(std::chrono::milliseconds& elapsed)
{
  auto remaining = server_timeout_ - elapsed;
  if (remaining <= std::chrono::milliseconds(0)) // 1: Faied to send goal becaseuse of timeout defined by user on Action Client.
  {
    future_goal_handle_.reset(); 
    this -> halt_bef();
    return false;
  }
  auto timeout = remaining > bt_loop_duration_ ? bt_loop_duration_ : remaining;
  auto result = callback_group_executor_.spin_until_future_complete(*future_goal_handle_, timeout);
  elapsed += timeout;
  if (result == rclcpp::FutureReturnCode::INTERRUPTED) // 2: Goal request was interrupted by errors caused on Action Client. (e.g., node shutdown or external interruption).
  {
    this -> halt_bef();
    future_goal_handle_.reset(); 
    throw std::runtime_error("send_goal failed");
  }
  if (result == rclcpp::FutureReturnCode::SUCCESS) // 3: Succeeded to send goal. Action Server received the goal and returned the goal response.
  {
    goal_handle_ = future_goal_handle_->get();
    future_goal_handle_.reset();
    if (!goal_handle_)
    {
      throw std::runtime_error("Goal was rejected by the action server");
    }
    return true;
  }
  // 4: Timed out while waiting for the Action Server to respond to the goal request. (rclcpp::FutureReturnCode::TIMEOUT)
  return false;
}



void LeafNodeBase::halt_bef() // Since goal_handle_ is invalid, all goals on the Action Client are canceled using async_cancel_all_goals().
{
    auto future_cancel = action_client_->async_cancel_all_goals();
    auto cancel_result = rclcpp::spin_until_future_complete(node_->get_node_base_interface(), future_cancel, std::chrono::seconds(1));

    if (cancel_result == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(node_->get_logger(), "All goals have been successfully cancelled.");
    } else if (cancel_result == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_WARN(node_->get_logger(), "Timeout occurred while cancelling all goals.");
      rclcpp::shutdown();
    } else if (cancel_result == rclcpp::FutureReturnCode::INTERRUPTED) {
      RCLCPP_ERROR(node_->get_logger(), "Cancellation was interrupted.");
      rclcpp::shutdown();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to cancel all goals.");
      rclcpp::shutdown();
    }
    setStatus(BT::NodeStatus::IDLE);
}



void LeafNodeBase::halt()
{
    if (should_cancel_goal())
    {
        RCLCPP_INFO(node_->get_logger(), "Attempting to cancel goal for %s", subtask_name_.c_str());
        auto future_cancel = action_client_->async_cancel_goal(goal_handle_);
        auto cancel_result = rclcpp::FutureReturnCode::TIMEOUT;

        while (true) 
        {
            cancel_result = callback_group_executor_.spin_until_future_complete(future_cancel, server_timeout_);

            if (cancel_result == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(node_->get_logger(), "Goal for %s successfully canceled.", subtask_name_.c_str());
                
                callback_group_executor_.spin_some(); 
                auto status = goal_handle_->get_status();
                if (status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
                {
                    RCLCPP_INFO(node_->get_logger(), "Server confirmed goal cancellation for %s", subtask_name_.c_str());
                    break; 
                }
                else
                {
                    RCLCPP_WARN(node_->get_logger(), "Server did not confirm cancellation, retrying...");
                    future_cancel = action_client_->async_cancel_goal(goal_handle_); 
                }
            }
            else if (cancel_result == rclcpp::FutureReturnCode::INTERRUPTED)
            {
                RCLCPP_WARN(node_->get_logger(), "Cancellation interrupted for %s. After %d trial, all nodes will be forcibly shutdown. ", subtask_name_.c_str(),cancel_process_count_);
                cancel_process_count_= cancel_process_count_ - 1;
                if (cancel_process_count_ == 0)
                {
                    RCLCPP_ERROR(node_->get_logger(), "All nodes will be forcibly shutdown. ");
                    rclcpp::shutdown();
                }
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(), "Waiting for goal cancellation to complete for %s...", subtask_name_.c_str());
            }
        }
    }
    else
    {
        RCLCPP_WARN(node_->get_logger(), "No active goal to cancel for %s", subtask_name_.c_str());
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
        RCLCPP_WARN(node_->get_logger(), "Timed out while waiting for subtask to acknowledge goal request for %s",subtask_name_.c_str());
        this->halt_bef();
        future_goal_handle_.reset();   

        return BT::NodeStatus::FAILURE;
      }
    }

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
          this->halt(); 

          if (!should_cancel_goal())
          {
              RCLCPP_INFO(node_->get_logger(), "Server confirmed cancellation or goal was already stopped.");
          }
          else
          {
              RCLCPP_WARN(node_->get_logger(), "Server failed to confirm cancellation.");
          }
          return BT::NodeStatus::FAILURE;
        }
      }

      callback_group_executor_.spin_some();


      if (!goal_result_available_)
      {
        return BT::NodeStatus::RUNNING;
      }
    }
  }
  catch (const std::runtime_error& e)
  {
    if (e.what() == std::string("send_goal failed") || e.what() == std::string("Goal was rejected by the action server"))
    {
      return BT::NodeStatus::FAILURE;
    }
    else
    {
      throw e;
    }
  }

  switch (result_.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Succeeded the Subtask");
      bt_status = NodeStatus::SUCCESS;
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(node_->get_logger(), "Aborted the Subtask");
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
