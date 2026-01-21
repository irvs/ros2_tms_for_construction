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

#include "tms_ts_primitive/Excavator/leaf_node.hpp"

using std::placeholders::_1;
using namespace BT;
using namespace std::chrono_literals;

NodeStatus LeafNodeExcavator::tick()
{
  if (status() == NodeStatus::IDLE)
  {
    setStatus(NodeStatus::RUNNING);
    should_send_goal_ = true;

    // previous_target_record_nameとtarget_record_nameの両方を取得
    Optional<std::string> previous_target_record_name = getInput<std::string>("previous_target_record_name");
    Optional<std::string> target_record_name = getInput<std::string>("target_record_name");
    
    // target_record_nameは必須
    if (!target_record_name) {
      RCLCPP_ERROR(node_->get_logger(),
        "target_record_name port not provided: %s", target_record_name.error().c_str());
      return NodeStatus::FAILURE;
    }
    
    // goal_.record_nameにはtarget_record_nameを設定
    goal_.record_name = target_record_name.value();
    
    // previous_target_record_nameをgoalに設定（オプション）
    if (previous_target_record_name) {
      goal_.previous_target_record_name = previous_target_record_name.value();
      RCLCPP_INFO(node_->get_logger(), "previous_target_record_name: %s", previous_target_record_name.value().c_str());
    } else {
      goal_.previous_target_record_name = "";  // 空文字列
    }
    RCLCPP_INFO(node_->get_logger(), "target_record_name: %s", target_record_name.value().c_str());

    if (!should_send_goal_)
    {
      return NodeStatus::FAILURE;
    }
    LeafNodeBase::send_new_goal();
  }

  // 以降は基底クラスのtick()と同じ処理
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
        RCLCPP_WARN(node_->get_logger(), "Timed out while waiting for primitive to acknowledge goal request for %s",primitive_name_.c_str());
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
                      primitive_name_.c_str());
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
      RCLCPP_INFO(node_->get_logger(), "Succeeded the Primitive");
      bt_status = NodeStatus::SUCCESS;
      break;

    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(node_->get_logger(), "Aborted the Primitive");
      bt_status = NodeStatus::FAILURE;
      break;

    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(node_->get_logger(), "Canceled the Primitive");
      bt_status = NodeStatus::FAILURE;
      break;

    default:
      throw std::logic_error("BtActionNode::Tick: invalid status value");
  }

  goal_handle_.reset();
  return bt_status;
}
