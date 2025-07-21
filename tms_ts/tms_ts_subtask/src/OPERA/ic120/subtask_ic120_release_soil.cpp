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

#include <vector>
#include "tms_ts_subtask/OPERA/ic120/subtask_ic120_release_soil.hpp"
// #include <glog/logging.h>

using std::placeholders::_1;
using std::placeholders::_2;

SubtaskIc120ReleaseSoil::SubtaskIc120ReleaseSoil() : SubtaskNodeBase("st_ic120_release_soil_node")
{
    this->action_server_ = rclcpp_action::create_server<tms_msg_ts::action::LeafNodeBase>(
        this, "st_ic120_release_soil",
        std::bind(&SubtaskIc120ReleaseSoil::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SubtaskIc120ReleaseSoil::handle_cancel, this, std::placeholders::_1),
        std::bind(&SubtaskIc120ReleaseSoil::handle_accepted, this, std::placeholders::_1));

    
    action_client_ = rclcpp_action::create_client<SetDumpAngle>(this, "set_dump_angle");
}

rclcpp_action::GoalResponse SubtaskIc120ReleaseSoil::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
{
    parameters = CustomGetParamFromDB<std::string, double>(goal->model_name, goal->record_name);
    if (parameters.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameters from DB");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SubtaskIc120ReleaseSoil::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel subtask node");
    if (client_future_goal_handle_.valid() &&
        client_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        auto goal_handle = client_future_goal_handle_.get();
        action_client_->async_cancel_goal(goal_handle);
    }
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SubtaskIc120ReleaseSoil::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    using namespace std::placeholders;
    std::thread{ std::bind(&SubtaskIc120ReleaseSoil::execute, this, _1), goal_handle }.detach();
}

void SubtaskIc120ReleaseSoil::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "subtask(st_ic120_release_soil) is executing...");
    auto result = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();
    auto handle_error = [&](const std::string& message) {
        if (goal_handle->is_active())
        {
        result->result = false;
        goal_handle->abort(result);
        RCLCPP_INFO(this->get_logger(), message.c_str());
        }
        else
        {
        RCLCPP_INFO(this->get_logger(), "Goal is not active");
        }
    };

    RCLCPP_INFO(this->get_logger(), "Get pose from DB.");

    auto goal_msg = SetDumpAngle::Goal();
    goal_msg.target_angle = parameters["target_angle"];


    //進捗状況を表示するFeedbackコールバックを設�?
    auto send_goal_options = rclcpp_action::Client<SetDumpAngle>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const auto& goal_handle) { goal_response_callback(goal_handle); };
    send_goal_options.feedback_callback = [this](const auto tmp, const auto feedback) {
        feedback_callback(tmp, feedback);
    };
    send_goal_options.result_callback = [this, goal_handle](const auto& result) { result_callback(goal_handle, result); };

    //Goal をサーバ�?�に送信
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void SubtaskIc120ReleaseSoil::goal_response_callback(const GoalHandleIc120ReleaseSoil::SharedPtr& goal_handle)
{
  if (!goal_handle)
  {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

  
void SubtaskIc120ReleaseSoil::feedback_callback(
    const GoalHandleIc120ReleaseSoil::SharedPtr,
    const std::shared_ptr<const GoalHandleIc120ReleaseSoil::Feedback> feedback)
{
  // TODO: Fix to feedback to leaf node
  // RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
}


//result
void SubtaskIc120ReleaseSoil::result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                                             const GoalHandleIc120ReleaseSoil::WrappedResult& result)
{
  if (!goal_handle->is_active())
  {
    RCLCPP_WARN(this->get_logger(), "Attempted to succeed an already succeeded goal");
    return;
  }

  auto result_to_leaf = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      result_to_leaf->result = true;
      goal_handle->succeed(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Subtask execution is succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      result_to_leaf->result = false;
      goal_handle->abort(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Subtask execution is aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      result_to_leaf->result = false;
      goal_handle->canceled(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Subtask execution is canceled");
      break;
    default:
      result_to_leaf->result = false;
      goal_handle->abort(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Unknown result code");
      break;
  }
}

int main(int argc, char* argv[])
{
    // Initialize Google's logging library.
    //   google::InitGoogleLogging(argv[0]);
    //   google::InstallFailureSignalHandler();

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubtaskIc120ReleaseSoil>());
    rclcpp::shutdown();
    return 0;
}
