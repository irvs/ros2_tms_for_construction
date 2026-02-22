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

#include "tms_ts_primitive/Excavator/primitive_excavator_change_pose_execute_from_plan.hpp"
#include <glog/logging.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

bool retimeRobotTrajectoryMsg(
  const rclcpp::Node::SharedPtr& node,
  moveit_msgs::msg::RobotTrajectory& traj_msg,
  const std::string& planning_group,
  double vel_scale,
  double acc_scale);

using namespace std::chrono_literals;

// Helper function to get numeric value from BSON element (supports int32, int64, and double)
namespace {
  double get_numeric_value(const bsoncxx::document::element& element) {
    switch (element.type()) {
      case bsoncxx::type::k_double:
        return element.get_double().value;
      case bsoncxx::type::k_int32:
        return static_cast<double>(element.get_int32().value);
      case bsoncxx::type::k_int64:
        return static_cast<double>(element.get_int64().value);
      default:
        throw std::runtime_error("Element is not a numeric type");
    }
  }
  
  double get_numeric_value(const bsoncxx::array::element& element) {
    switch (element.type()) {
      case bsoncxx::type::k_double:
        return element.get_double().value;
      case bsoncxx::type::k_int32:
        return static_cast<double>(element.get_int32().value);
      case bsoncxx::type::k_int64:
        return static_cast<double>(element.get_int64().value);
      default:
        throw std::runtime_error("Array element is not a numeric type");
    }
  }

  double get_scale_from_db_json(
    const std::map<std::string, std::string>& param_from_db,
    const std::string& key,
    const std::string& field_name,
    double default_value)
  {
    auto it = param_from_db.find(key);
    if (it == param_from_db.end()) {
      return default_value;
    }

    try {
      auto doc = bsoncxx::from_json(it->second);
      auto view = doc.view();
      if (!view[field_name]) {
        return default_value;
      }
      return get_numeric_value(view[field_name]);  // int/double対応済み
    } catch (const std::exception& e) {
      // 壊れてたらデフォルトにフォールバック
      return default_value;
    }
  }
}

PrimitiveExcavatorChangePoseExecuteFromPlan::PrimitiveExcavatorChangePoseExecuteFromPlan() : PrimitiveNodeBase("primitive_excavator_change_pose_execute_from_plan_node")
{
    auto options_server = rcl_action_server_get_default_options();
    options_server.goal_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
    options_server.result_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
    options_server.cancel_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
    options_server.feedback_topic_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
    options_server.status_topic_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();

    auto options_client = rcl_action_client_get_default_options();
    options_client.goal_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
    options_client.result_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
    options_client.cancel_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
    options_client.feedback_topic_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
    options_client.status_topic_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  
  action_server_ = rclcpp_action::create_server<tms_msg_ts::action::LeafNodeBase>(
      this, "primitive_excavator_change_pose_execute_from_plan_retime",
      std::bind(&PrimitiveExcavatorChangePoseExecuteFromPlan::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PrimitiveExcavatorChangePoseExecuteFromPlan::handle_cancel, this, std::placeholders::_1),
      std::bind(&PrimitiveExcavatorChangePoseExecuteFromPlan::handle_accepted, this, std::placeholders::_1),
      options_server);

  action_client_ = rclcpp_action::create_client<TmsRpExcavator>(this, "tms_rp_excavator", nullptr, options_client);
  if (action_client_->wait_for_action_server())
  {
    RCLCPP_INFO(this->get_logger(), "Action server is ready");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
  }
}

rclcpp_action::GoalResponse PrimitiveExcavatorChangePoseExecuteFromPlan::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
{
  used_model_name_ = goal->model_name;
  used_record_name_ = goal->record_name;
  param_from_db_ = GetParamFromDBAsJson(goal->model_name, goal->record_name);
  if (param_from_db_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameters from DB");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PrimitiveExcavatorChangePoseExecuteFromPlan::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel primitive node");
  if (client_future_goal_handle_.valid() &&
      client_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
  {
    auto goal_handle = client_future_goal_handle_.get();
    action_client_->async_cancel_goal(goal_handle);
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PrimitiveExcavatorChangePoseExecuteFromPlan::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  using namespace std::placeholders;
  std::thread{ std::bind(&PrimitiveExcavatorChangePoseExecuteFromPlan::execute, this, _1), goal_handle }.detach();
}

void PrimitiveExcavatorChangePoseExecuteFromPlan::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto result = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();

  // Function for error handling
  auto handle_error = [&](const std::string& message) {
    if (goal_handle->is_active())
    {
      result->result = false;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal is not active");
    }
  };
  
  if (!action_client_->action_server_is_ready())
  {
    handle_error("Action server not available");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Getting plan from DB");
  RCLCPP_INFO(this->get_logger(), "param_from_db_ contents:");
  for (const auto& [key, value] : param_from_db_)
  {
    RCLCPP_INFO(this->get_logger(), "  %s: %s", key.c_str(), value.c_str());
  }

  // TmsRpExcavatorのゴールメッセージを作成
  auto goal_msg = TmsRpExcavator::Goal();
  goal_msg.command = TmsRpExcavator::Goal::CMD_EXECUTE_PLAN;
  
  // planning_groupを取得
  if (param_from_db_.count("planning_group")) {
    try {
      auto doc = bsoncxx::from_json(param_from_db_["planning_group"]);
      auto view = doc.view();
      if (view["planning_group"] && view["planning_group"].type() == bsoncxx::type::k_string) {
        goal_msg.planning_group = view["planning_group"].get_string().value.to_string();
        RCLCPP_INFO(this->get_logger(), "Planning group: %s", goal_msg.planning_group.c_str());
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse planning_group: %s", e.what());
    }
  }
  
  if (goal_msg.planning_group.empty()) {
    handle_error("planning_group is required but not found in DB");
    return;
  }

  // scaling をDBから取得（なければ1.0）
  double velocity_scaling = get_scale_from_db_json(param_from_db_, "velocity_scaling", "velocity_scaling", 1.0);
  double acceleration_scaling      = get_scale_from_db_json(param_from_db_, "acceleration_scaling",      "acceleration_scaling",      1.0);

  if (velocity_scaling <= 0.0 || velocity_scaling > 1.0 || acceleration_scaling <= 0.0 || acceleration_scaling > 1.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid scaling factors from DB: velocity_scaling=%.3f, acceleration_scaling=%.3f. Must be in (0.0, 1.0].", velocity_scaling, acceleration_scaling);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Scaling from DB: velocity_scale=%.3f acc_scale=%.3f",
              velocity_scaling, acceleration_scaling);

  // データベースからplanを取得してRobotTrajectory配列に変換
  try {
    if (!param_from_db_.count("plan")) {
      RCLCPP_ERROR(this->get_logger(), "Missing required parameter: plan");
      handle_error("Missing required parameter: plan");
      return;
    }

    auto doc = bsoncxx::from_json(param_from_db_["plan"]);
    auto view = doc.view();
    
    if (!view["plan"]) {
      RCLCPP_ERROR(this->get_logger(), "plan field not found in JSON");
      handle_error("plan field not found");
      return;
    }
    
    auto plan_element = view["plan"];
    
    if (plan_element.type() != bsoncxx::type::k_document) {
      RCLCPP_ERROR(this->get_logger(), "plan must be a document");
      handle_error("plan must be a document");
      return;
    }
    
    auto plan_doc = plan_element.get_document().value;
    
    // 各プラン（"1", "2", "3"...）を処理
    for (auto&& plan_element : plan_doc) {
      std::string plan_key = plan_element.key().to_string();
      RCLCPP_INFO(this->get_logger(), "Processing plan: %s", plan_key.c_str());
      
      if (plan_element.type() != bsoncxx::type::k_document) {
        RCLCPP_WARN(this->get_logger(), "Plan element %s is not a document, skipping", plan_key.c_str());
        continue;
      }
      
      auto trajectory_doc = plan_element.get_document().value;
      moveit_msgs::msg::RobotTrajectory robot_trajectory;
      
      // joint_trajectoryの変換
      if (trajectory_doc["joint_trajectory"]) {
        auto joint_traj_doc = trajectory_doc["joint_trajectory"].get_document().value;
        
        // joint_names
        if (joint_traj_doc["joint_names"]) {
          auto joint_names_array = joint_traj_doc["joint_names"].get_array().value;
          for (auto&& name : joint_names_array) {
            robot_trajectory.joint_trajectory.joint_names.push_back(
              name.get_string().value.to_string()
            );
          }
        }
        
        // points
        if (joint_traj_doc["points"]) {
          auto points_array = joint_traj_doc["points"].get_array().value;
          for (auto&& point_element : points_array) {
            auto point_doc = point_element.get_document().value;
            trajectory_msgs::msg::JointTrajectoryPoint point;
            
            // positions
            if (point_doc["positions"]) {
              auto positions_array = point_doc["positions"].get_array().value;
              for (auto&& pos : positions_array) {
                point.positions.push_back(get_numeric_value(pos));
              }
            }
            
            // velocities
            if (point_doc["velocities"]) {
              auto velocities_array = point_doc["velocities"].get_array().value;
              for (auto&& vel : velocities_array) {
                point.velocities.push_back(get_numeric_value(vel));
              }
            }
            
            // accelerations
            if (point_doc["accelerations"]) {
              auto accelerations_array = point_doc["accelerations"].get_array().value;
              for (auto&& acc : accelerations_array) {
                point.accelerations.push_back(get_numeric_value(acc));
              }
            }
            
            // time_from_start
            if (point_doc["time_from_start"]) {
              auto time_doc = point_doc["time_from_start"].get_document().value;
              if (time_doc["sec"]) {
                point.time_from_start.sec = time_doc["sec"].get_int32().value;
              }
              if (time_doc["nanosec"]) {
                point.time_from_start.nanosec = static_cast<uint32_t>(time_doc["nanosec"].get_int32().value);
              }
            }
            
            robot_trajectory.joint_trajectory.points.push_back(point);
          }
        }
      }
      
      // multi_dof_joint_trajectoryの変換
      if (trajectory_doc["multi_dof_joint_trajectory"]) {
        auto multi_dof_doc = trajectory_doc["multi_dof_joint_trajectory"].get_document().value;
        
        // joint_names
        if (multi_dof_doc["joint_names"]) {
          auto joint_names_array = multi_dof_doc["joint_names"].get_array().value;
          for (auto&& name : joint_names_array) {
            robot_trajectory.multi_dof_joint_trajectory.joint_names.push_back(
              name.get_string().value.to_string()
            );
          }
        }
        
        // points
        if (multi_dof_doc["points"]) {
          auto points_array = multi_dof_doc["points"].get_array().value;
          for (auto&& point_element : points_array) {
            auto point_doc = point_element.get_document().value;
            trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
            
            // transforms
            if (point_doc["transforms"]) {
              auto transforms_array = point_doc["transforms"].get_array().value;
              for (auto&& transform_element : transforms_array) {
                auto transform_doc = transform_element.get_document().value;
                geometry_msgs::msg::Transform transform;
                
                // translation
                if (transform_doc["translation"]) {
                  auto trans_doc = transform_doc["translation"].get_document().value;
                  if (trans_doc["x"]) transform.translation.x = get_numeric_value(trans_doc["x"]);
                  if (trans_doc["y"]) transform.translation.y = get_numeric_value(trans_doc["y"]);
                  if (trans_doc["z"]) transform.translation.z = get_numeric_value(trans_doc["z"]);
                }
                
                // rotation
                if (transform_doc["rotation"]) {
                  auto rot_doc = transform_doc["rotation"].get_document().value;
                  if (rot_doc["x"]) transform.rotation.x = get_numeric_value(rot_doc["x"]);
                  if (rot_doc["y"]) transform.rotation.y = get_numeric_value(rot_doc["y"]);
                  if (rot_doc["z"]) transform.rotation.z = get_numeric_value(rot_doc["z"]);
                  if (rot_doc["w"]) transform.rotation.w = get_numeric_value(rot_doc["w"]);
                }
                
                point.transforms.push_back(transform);
              }
            }
            
            // velocities
            if (point_doc["velocities"]) {
              auto velocities_array = point_doc["velocities"].get_array().value;
              for (auto&& vel_element : velocities_array) {
                auto vel_doc = vel_element.get_document().value;
                geometry_msgs::msg::Twist twist;
                
                // linear
                if (vel_doc["linear"]) {
                  auto lin_doc = vel_doc["linear"].get_document().value;
                  if (lin_doc["x"]) twist.linear.x = get_numeric_value(lin_doc["x"]);
                  if (lin_doc["y"]) twist.linear.y = get_numeric_value(lin_doc["y"]);
                  if (lin_doc["z"]) twist.linear.z = get_numeric_value(lin_doc["z"]);
                }
                
                // angular
                if (vel_doc["angular"]) {
                  auto ang_doc = vel_doc["angular"].get_document().value;
                  if (ang_doc["x"]) twist.angular.x = get_numeric_value(ang_doc["x"]);
                  if (ang_doc["y"]) twist.angular.y = get_numeric_value(ang_doc["y"]);
                  if (ang_doc["z"]) twist.angular.z = get_numeric_value(ang_doc["z"]);
                }
                
                point.velocities.push_back(twist);
              }
            }
            
            // accelerations
            if (point_doc["accelerations"]) {
              auto accelerations_array = point_doc["accelerations"].get_array().value;
              for (auto&& acc_element : accelerations_array) {
                auto acc_doc = acc_element.get_document().value;
                geometry_msgs::msg::Twist twist;
                
                // linear
                if (acc_doc["linear"]) {
                  auto lin_doc = acc_doc["linear"].get_document().value;
                  if (lin_doc["x"]) twist.linear.x = get_numeric_value(lin_doc["x"]);
                  if (lin_doc["y"]) twist.linear.y = get_numeric_value(lin_doc["y"]);
                  if (lin_doc["z"]) twist.linear.z = get_numeric_value(lin_doc["z"]);
                }
                
                // angular
                if (acc_doc["angular"]) {
                  auto ang_doc = acc_doc["angular"].get_document().value;
                  if (ang_doc["x"]) twist.angular.x = get_numeric_value(ang_doc["x"]);
                  if (ang_doc["y"]) twist.angular.y = get_numeric_value(ang_doc["y"]);
                  if (ang_doc["z"]) twist.angular.z = get_numeric_value(ang_doc["z"]);
                }
                
                point.accelerations.push_back(twist);
              }
            }
            
            // time_from_start
            if (point_doc["time_from_start"]) {
              auto time_doc = point_doc["time_from_start"].get_document().value;
              if (time_doc["sec"]) {
                point.time_from_start.sec = time_doc["sec"].get_int32().value;
              }
              if (time_doc["nanosec"]) {
                point.time_from_start.nanosec = static_cast<uint32_t>(time_doc["nanosec"].get_int32().value);
              }
            }
            
            robot_trajectory.multi_dof_joint_trajectory.points.push_back(point);
          }
        }
      }

      for (auto& p : robot_trajectory.joint_trajectory.points) {
        p.velocities.clear();
        p.accelerations.clear();
        p.effort.clear();
      }
      
      if (!retimeRobotTrajectoryMsg(shared_from_this(), robot_trajectory, goal_msg.planning_group, velocity_scaling, acceleration_scaling)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to retime trajectory %s", plan_key.c_str());
        handle_error("Failed to retime trajectory");
        return;
      }      
      
      goal_msg.plan.push_back(robot_trajectory);
      RCLCPP_INFO(this->get_logger(), "Added plan %s with %zu joint trajectory points", 
                  plan_key.c_str(), 
                  robot_trajectory.joint_trajectory.points.size());
    }
    
    if (goal_msg.plan.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No valid plans were added");
      handle_error("No valid plans in plan field");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu plan(s) from database", goal_msg.plan.size());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse plan: %s", e.what());
    handle_error("Failed to parse plan from DB");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Sending EXECUTE_PLAN command to tms_rp_excavator");

  // Send goal to TMS_IF
  auto send_goal_options = rclcpp_action::Client<TmsRpExcavator>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](const auto& goal_handle) { goal_response_callback(goal_handle); };
  send_goal_options.feedback_callback = [this](const auto tmp, const auto feedback) {
    feedback_callback(tmp, feedback);
  };
  send_goal_options.result_callback = [this, goal_handle](const auto& result) { result_callback(goal_handle, result); };

  client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void PrimitiveExcavatorChangePoseExecuteFromPlan::goal_response_callback(const GoalHandleTmsRpExcavator::SharedPtr& goal_handle)
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

void PrimitiveExcavatorChangePoseExecuteFromPlan::feedback_callback(
    const GoalHandleTmsRpExcavator::SharedPtr,
    const std::shared_ptr<const TmsRpExcavator::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Feedback: %s (progress: %.2f)", feedback->state.c_str(), feedback->progress);
}

void PrimitiveExcavatorChangePoseExecuteFromPlan::result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                                             const GoalHandleTmsRpExcavator::WrappedResult& result)
{
  if (!goal_handle->is_active())
  {
    RCLCPP_WARN(this->get_logger(), "Attempted to complete an already completed goal");
    return;
  }

  auto result_to_leaf = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();
  
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      if (result.result->success) {
        result_to_leaf->result = true;
        goal_handle->succeed(result_to_leaf);
        RCLCPP_INFO(this->get_logger(), "Plan execution succeeded: %s", result.result->message.c_str());
      } else {
        result_to_leaf->result = false;
        goal_handle->abort(result_to_leaf);
        RCLCPP_ERROR(this->get_logger(), "Plan execution failed: %s (error code: %d)", 
                     result.result->message.c_str(), result.result->moveit_error_code);
      }
      break;
      
    case rclcpp_action::ResultCode::ABORTED:
      result_to_leaf->result = false;
      goal_handle->abort(result_to_leaf);
      RCLCPP_ERROR(this->get_logger(), "Plan execution was aborted");
      break;
      
    case rclcpp_action::ResultCode::CANCELED:
      result_to_leaf->result = false;
      goal_handle->canceled(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Plan execution was canceled");
      break;
      
    default:
      result_to_leaf->result = false;
      goal_handle->abort(result_to_leaf);
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }
}
bool retimeRobotTrajectoryMsg(
  const rclcpp::Node::SharedPtr& node,
  moveit_msgs::msg::RobotTrajectory& traj_msg,
  const std::string& planning_group,
  double vel_scale, double acc_scale)
{
  robot_model_loader::RobotModelLoader loader(node, "robot_description");
  moveit::core::RobotModelConstPtr model = loader.getModel();
  if (!model) return false;

  const moveit::core::JointModelGroup* jmg = model->getJointModelGroup(planning_group);
  if (!jmg) return false;

  // reference_state（開始状態）を用意
  moveit::core::RobotState reference_state(model);
  reference_state.setToDefaultValues();

  // 先頭点の positions を reference_state に反映（joint_names と同じ順番前提）
  if (!traj_msg.joint_trajectory.points.empty() &&
      traj_msg.joint_trajectory.joint_names.size() == traj_msg.joint_trajectory.points.front().positions.size())
  {
    for (size_t i = 0; i < traj_msg.joint_trajectory.joint_names.size(); ++i)
    {
      reference_state.setVariablePosition(
        traj_msg.joint_trajectory.joint_names[i],
        traj_msg.joint_trajectory.points.front().positions[i]);
    }
  }
  reference_state.update();

  // msg -> RobotTrajectory
  robot_trajectory::RobotTrajectory rt(model, jmg);
  rt.setRobotTrajectoryMsg(reference_state, traj_msg);

  auto dur_to_sec = [](const builtin_interfaces::msg::Duration& d) {
    return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
  };
  
  if (!traj_msg.joint_trajectory.points.empty()) {
    const auto& t0 = traj_msg.joint_trajectory.points.front().time_from_start;
    const auto& tN = traj_msg.joint_trajectory.points.back().time_from_start;
    RCLCPP_INFO(node->get_logger(), "[BEFORE] t0=%.6f tN=%.6f (N=%zu)",
                dur_to_sec(t0), dur_to_sec(tN), traj_msg.joint_trajectory.points.size());
  }

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  const bool ok = totg.computeTimeStamps(rt, vel_scale, acc_scale);

  if (!traj_msg.joint_trajectory.points.empty()) {
    const auto& t0 = traj_msg.joint_trajectory.points.front().time_from_start;
    const auto& tN = traj_msg.joint_trajectory.points.back().time_from_start;
    RCLCPP_INFO(node->get_logger(), "[AFTER ] t0=%.6f tN=%.6f",
                dur_to_sec(t0), dur_to_sec(tN));
  }
  
  // ↑ シグネチャ/意味 :contentReference[oaicite:6]{index=6}
  if (!ok) return false;

  // RobotTrajectory -> msg
  moveit_msgs::msg::RobotTrajectory out;
  rt.getRobotTrajectoryMsg(out);
  traj_msg = out;
  return true;
}
/*******************/

int main(int argc, char* argv[])
{
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrimitiveExcavatorChangePoseExecuteFromPlan>());
  rclcpp::shutdown();
  return 0;
}
