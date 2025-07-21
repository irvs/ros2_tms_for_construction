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
#include "tms_ts_subtask/OPERA/mst110cr/subtask_mst110cr_navigate_through_poses.hpp"
// #include <glog/logging.h>

using std::placeholders::_1;
using std::placeholders::_2;

SubtaskMst110crNavigateThroughPoses::SubtaskMst110crNavigateThroughPoses() : SubtaskNodeBase("st_mst110cr_navigate_through_poses_node")
{
    this->action_server_ = rclcpp_action::create_server<tms_msg_ts::action::LeafNodeBase>(
        this, "st_mst110cr_navigate_through_poses",
        std::bind(&SubtaskMst110crNavigateThroughPoses::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&SubtaskMst110crNavigateThroughPoses::handle_cancel, this, std::placeholders::_1),
        std::bind(&SubtaskMst110crNavigateThroughPoses::handle_accepted, this, std::placeholders::_1));

    
    action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
}

rclcpp_action::GoalResponse SubtaskMst110crNavigateThroughPoses::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
{
    parameters = CustomGetParamFromDB<std::pair<std::string, std::string>, double>(goal->model_name, goal->record_name);
    if (parameters.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get parameters from DB");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SubtaskMst110crNavigateThroughPoses::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
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

void SubtaskMst110crNavigateThroughPoses::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    using namespace std::placeholders;
    std::thread{ std::bind(&SubtaskMst110crNavigateThroughPoses::execute, this, _1), goal_handle }.detach();
}

void SubtaskMst110crNavigateThroughPoses::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "subtask(st_mst110cr_navigate_through_poses) is executing...");
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

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    auto goal_msg = NavigateThroughPoses::Goal();


    int point_num = parameters.size() / 7;
    std::cout << "Total number of points: " << parameters.size() << std::endl;
    std::cout << "point_num: " << point_num << std::endl;
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();

    for (int i=0; i < point_num; i++){
      pose.pose.position.x = parameters[std::make_pair("x",std::to_string(i))];
      pose.pose.position.y = parameters[std::make_pair("y",std::to_string(i))];
      pose.pose.position.z = parameters[std::make_pair("z",std::to_string(i))];
      pose.pose.orientation.x = parameters[std::make_pair("qx",std::to_string(i))];
      pose.pose.orientation.y = parameters[std::make_pair("qy",std::to_string(i))];
      pose.pose.orientation.z = parameters[std::make_pair("qz",std::to_string(i))];
      pose.pose.orientation.w = parameters[std::make_pair("qw",std::to_string(i))];
      poses.push_back(pose);
      std::cout << "Point " << i << ": " << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z << std::endl;
      std::cout << "Pose " << i << ": " << pose.pose.orientation.x << ", " << pose.pose.orientation.y << ", " << pose.pose.orientation.z << ", " << pose.pose.orientation.w << std::endl;
    }
    goal_msg.poses = poses;


    //進捗状況を表示するFeedbackコールバックを設�?
    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const auto& goal_handle) { goal_response_callback(goal_handle); };
    send_goal_options.feedback_callback = [this](const auto tmp, const auto feedback) {
        feedback_callback(tmp, feedback);
    };
    send_goal_options.result_callback = [this, goal_handle](const auto& result) { result_callback(goal_handle, result); };

    //Goal をサーバ�?�に送信
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void SubtaskMst110crNavigateThroughPoses::goal_response_callback(const GoalHandleMst110crNavigateThroughPoses::SharedPtr& goal_handle)
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

  
void SubtaskMst110crNavigateThroughPoses::feedback_callback(
    const GoalHandleMst110crNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const GoalHandleMst110crNavigateThroughPoses::Feedback> feedback)
{
  // TODO: Fix to feedback to leaf node
  // RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
}


//result
void SubtaskMst110crNavigateThroughPoses::result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                                             const GoalHandleMst110crNavigateThroughPoses::WrappedResult& result)
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
    rclcpp::spin(std::make_shared<SubtaskMst110crNavigateThroughPoses>());
    rclcpp::shutdown();
    return 0;
}
