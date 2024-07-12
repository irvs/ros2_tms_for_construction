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

#include "tms_ts_subtask/zx200/subtask_zx200_change_pose.hpp"
#include <glog/logging.h>

using namespace std::chrono_literals;

SubtaskZx200ChangePose::SubtaskZx200ChangePose() : SubtaskNodeBase("subtask_zx200_change_pose_node")
{
  action_server_ = rclcpp_action::create_server<tms_msg_ts::action::LeafNodeBase>(
      this, "subtask_zx200_change_pose",
      std::bind(&SubtaskZx200ChangePose::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SubtaskZx200ChangePose::handle_cancel, this, std::placeholders::_1),
      std::bind(&SubtaskZx200ChangePose::handle_accepted, this, std::placeholders::_1));

  action_client_ = rclcpp_action::create_client<Zx200ChangePose>(this, "tms_rp_zx200_change_pose");
  if (action_client_->wait_for_action_server())
  {
    RCLCPP_INFO(this->get_logger(), "Action server is ready");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
  }
}

// std::map<std::string, double> SubtaskZx200ChangePose::GetParamFromDB(std::string model_name, std::string record_name)
// {
//   // Connect to MongoDB
//   mongocxx::client client{ mongocxx::uri{ "mongodb://localhost:27017" } };
//   mongocxx::database db = client["rostmsdb"];
//   mongocxx::collection collection = db["parameter"];

//   // Query to MongoDB
//   bsoncxx::builder::stream::document filter_builder;
//   filter_builder << "model_name" << model_name << "record_name" << record_name;
//   auto filter = filter_builder.view();
//   auto result = collection.find_one(filter);
//   if (result)
//   {
//     std::map<std::string, double> dataMap;

//     for (auto&& element : result->view())
//     {
//       std::string key = element.key().to_string();
//       if (key != "_id" && key != "model_name" && key != "type" && key != "record_name")
//       {
//         if (element.type() == bsoncxx::type::k_double)
//         {
//           double value = static_cast<double>(element.get_double());
//           dataMap[key] = value;
//         }
//         else if (element.type() == bsoncxx::type::k_int32)
//         {
//           double value = static_cast<double>(element.get_int32().value);
//           dataMap[key] = value;
//         }
//         else if (element.type() == bsoncxx::type::k_int64)
//         {
//           double value = static_cast<double>(element.get_int64().value);
//           dataMap[key] = value;
//         }
//       }
//     }

//     return dataMap;
//   }
//   else
//   {
//     std::cout << "Dynamic parameter not found in your parameter collection" << std::endl;
//     return std::map<std::string, double>();
//   }
// }

rclcpp_action::GoalResponse SubtaskZx200ChangePose::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
{
  param_from_db_ = CustomGetParamFromDB<std::string, double>(goal->model_name, goal->record_name);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SubtaskZx200ChangePose::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
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

void SubtaskZx200ChangePose::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  using namespace std::placeholders;
  std::thread{ std::bind(&SubtaskZx200ChangePose::execute, this, _1), goal_handle }.detach();
}

void SubtaskZx200ChangePose::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto result = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();

  // Function for error handling
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

  // RCLCPP_INFO(this->get_logger(), "subtask is executing...");

  if (!action_client_->action_server_is_ready())
  {
    handle_error("Action server not available");
    return;
  }

  // TODO: Fix to get array from DB
  auto goal_msg = Zx200ChangePose::Goal();
  if (param_from_db_["trajectory"] > 0.5)
  {
    RCLCPP_INFO(this->get_logger(), "Get trajectory from DB.");

    goal_msg.trajectory.joint_names = { "swing_joint", "boom_joint", "arm_joint", "bucket_joint", "bucket_end_joint" };
    trajectory_msgs::msg::JointTrajectoryPoint target_pose;
    for (const auto& joint_name : goal_msg.trajectory.joint_names)
    {
      target_pose.positions.push_back(param_from_db_[joint_name]);
    }

    goal_msg.trajectory.points.push_back(target_pose);
  }
  else if (param_from_db_["pose"] > 0.5)
  {
    RCLCPP_INFO(this->get_logger(), "Get pose from DB.");

    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = param_from_db_["x"];
    target_pose.position.y = param_from_db_["y"];
    target_pose.position.z = param_from_db_["z"];
    target_pose.orientation.x = param_from_db_["qx"];
    target_pose.orientation.y = param_from_db_["qy"];
    target_pose.orientation.z = param_from_db_["qz"];
    target_pose.orientation.w = param_from_db_["qw"];

    goal_msg.pose_sequence.push_back(target_pose);
  }
  else if (param_from_db_["position_with_angle"] > 0.5)
  {
    RCLCPP_INFO(this->get_logger(), "Get position_with_angle from DB.");

    tms_msg_rp::msg::TmsRpZx200PositionWithAngle target_pose;
    target_pose.position.x = param_from_db_["x"];
    target_pose.position.y = param_from_db_["y"];
    target_pose.position.z = param_from_db_["z"];
    target_pose.theta_w = param_from_db_["theta_w"];

    goal_msg.position_with_angle_sequence.push_back(target_pose);
  }
  else
  {
    handle_error("No trajectory, pose, or position_with_angle found in DB");
    return;
  }

  // Send goal to TMS_RP
  auto send_goal_options = rclcpp_action::Client<Zx200ChangePose>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](const auto& goal_handle) { goal_response_callback(goal_handle); };
  send_goal_options.feedback_callback = [this](const auto tmp, const auto feedback) {
    feedback_callback(tmp, feedback);
  };
  send_goal_options.result_callback = [this, goal_handle](const auto& result) { result_callback(goal_handle, result); };

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void SubtaskZx200ChangePose::goal_response_callback(const GoalHandleZx200ChangePose::SharedPtr& goal_handle)
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

void SubtaskZx200ChangePose::feedback_callback(
    const GoalHandleZx200ChangePose::SharedPtr,
    const std::shared_ptr<const GoalHandleZx200ChangePose::Feedback> feedback)
{
  // TODO: Fix to feedback to leaf node
  RCLCPP_INFO(this->get_logger(), "Feedback received: %s", feedback->state.c_str());
}

void SubtaskZx200ChangePose::result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                                             const GoalHandleZx200ChangePose::WrappedResult& result)
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
/*******************/

int main(int argc, char* argv[])
{
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubtaskZx200ChangePose>());
  rclcpp::shutdown();
  return 0;
}
