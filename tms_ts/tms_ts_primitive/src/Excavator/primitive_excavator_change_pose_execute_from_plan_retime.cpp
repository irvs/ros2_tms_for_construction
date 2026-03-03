// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.

//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at

//      http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "tms_ts_primitive/Excavator/primitive_excavator_change_pose_execute_from_plan_retime.hpp"
#include <glog/logging.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

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

  double duration_to_sec(const builtin_interfaces::msg::Duration& d)
{
  return static_cast<double>(d.sec) + 1e-9 * static_cast<double>(d.nanosec);
}

builtin_interfaces::msg::Duration sec_to_duration(double t)
{
  if (t < 0.0) t = 0.0;
  builtin_interfaces::msg::Duration d;
  d.sec = static_cast<int32_t>(std::floor(t));
  const double frac = t - static_cast<double>(d.sec);
  int64_t ns = static_cast<int64_t>(std::llround(frac * 1e9));
  // 丸めで1e9になった場合の繰り上がり
  if (ns >= 1000000000LL) { d.sec += 1; ns -= 1000000000LL; }
  if (ns < 0) ns = 0;
  d.nanosec = static_cast<uint32_t>(ns);
  return d;
}

// time_from_start を一律倍率でスケールする（pointsの形は変えない）
bool scale_trajectory_time(moveit_msgs::msg::RobotTrajectory& traj, double time_scale)
{
  if (time_scale <= 0.0) return false;

  if (time_scale == 1.0) {
    // スケーリングなしなら何もしない
    return true;
  }

  for (auto& p : traj.joint_trajectory.points) {
    p.velocities.clear();
    p.accelerations.clear();
    p.effort.clear();
  }

  auto& jt = traj.joint_trajectory;
  if (jt.points.empty()) return true;

  // 単調増加を保つ（0や逆転があるとcontrollerが嫌がることがある）
  double last_t = -1.0;

  for (auto& p : jt.points) {
    double t = duration_to_sec(p.time_from_start);
    t *= time_scale;

    // 安全のため単調増加に補正（同一時刻が並ぶとまずいことがある）
    if (t <= last_t) {
      t = last_t + 1e-6;  // 1µs
    }
    p.time_from_start = sec_to_duration(t);
    last_t = t;

    // 速度/加速度などは送らない（既にclearしてるが念のため）
    p.velocities.clear();
    p.accelerations.clear();
    p.effort.clear();
  }
  return true;
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

  traj_action_client_ = rclcpp_action::create_client<traj_recorder_msgs::action::TrajFollow>(this, "traj_follow_record", nullptr, options_client);
  if (traj_action_client_->wait_for_action_server())
  {
    RCLCPP_INFO(this->get_logger(), "Trajectory action server is ready");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Trajectory action server not available after waiting");
  }

}

rclcpp_action::GoalResponse PrimitiveExcavatorChangePoseExecuteFromPlan::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
{
  used_model_name_ = goal->model_name;
  used_record_name_ = goal->record_name;
  
  // record_nameをカンマ区切りで分解
  std::vector<std::string> record_names;
  std::stringstream ss(goal->record_name);
  std::string record_name;
  while (std::getline(ss, record_name, ',')) {
    // 前後の空白を削除
    record_name.erase(0, record_name.find_first_not_of(" \t\n\r\f\v"));
    record_name.erase(record_name.find_last_not_of(" \t\n\r\f\v") + 1);
    if (!record_name.empty()) {
      record_names.push_back(record_name);
    }
  }
  
  if (record_names.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No valid record names provided");
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  RCLCPP_INFO(this->get_logger(), "Processing %zu record names", record_names.size());
  
  // 各record_nameからパラメータを取得
  params_from_db_.clear();
  for (const auto& rname : record_names) {
    auto params = GetParamFromDBAsJson(goal->model_name, rname);
    if (params.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get parameters from DB for record: %s", rname.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    params_from_db_.push_back(params);
    RCLCPP_INFO(this->get_logger(), "Loaded parameters for record: %s", rname.c_str());
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
  };
  
  if (!action_client_->action_server_is_ready())
  {
    handle_error("Action server not available");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loading plans from %zu record(s)", params_from_db_.size());

  // TmsRpExcavatorのゴールメッセージを作成
  auto goal_msg = TmsRpExcavator::Goal();
  goal_msg.command = TmsRpExcavator::Goal::CMD_EXECUTE_PLAN;
  
  // 最初のレコードからplanning_groupを取得
  if (params_from_db_.empty()) {
    handle_error("No parameters loaded from database");
    return;
  }
  
  const auto& first_param = params_from_db_[0];
  if (first_param.count("planning_group")) {
    try {
      auto doc = bsoncxx::from_json(first_param.at("planning_group"));
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

  // 最初のレコードからtime_scaleを取得
  double time_scale = get_scale_from_db_json(first_param, "time_scale", "time_scale", 1.0);
  double velocity_scale = get_scale_from_db_json(first_param, "velocity_scale", "velocity_scale", 0.0);
  double acceleration_scale = get_scale_from_db_json(first_param, "acceleration_scale", "acceleration_scale", 0.0);

  if (time_scale < 1.0 && velocity_scale > 1.0 && acceleration_scale > 1.0) {
    RCLCPP_ERROR(this->get_logger(), "Time scaling from DB is less than 1.0: time_scale=%.3f. This may cause the trajectory to be executed faster than planned, which can be dangerous. Please ensure that time_scale is set appropriately.", time_scale);
    return;
  }

  if (time_scale <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid time_scale from DB: %.3f. Must be > 0.", time_scale);
    return;
  }
  
  const double speed_scale = 1.0 / time_scale;
  RCLCPP_INFO(this->get_logger(), "Time scaling from DB: time_scale=%.3f => speed_scale=%.3f",
              time_scale, speed_scale);

  // 各record_nameからplanを取得してRobotTrajectory配列に変換
  std::vector<moveit_msgs::msg::RobotTrajectory> loaded_plans;
  try {
    for (size_t record_idx = 0; record_idx < params_from_db_.size(); ++record_idx) {
      const auto& param_from_db = params_from_db_[record_idx];
      
      if (!param_from_db.count("plan")) {
        RCLCPP_WARN(this->get_logger(), "Record %zu missing plan field, skipping", record_idx);
        continue;
      }

      auto doc = bsoncxx::from_json(param_from_db.at("plan"));
      auto view = doc.view();
      
      // "plan"キーでラップされているかチェック
      auto plan_view = view;
      if (view["plan"] && view["plan"].type() == bsoncxx::type::k_document) {
        plan_view = view["plan"].get_document().value;
      }
      
      // 単一のRobotTrajectoryとして読み込む
      moveit_msgs::msg::RobotTrajectory robot_trajectory;
      
      // joint_trajectoryの変換
      if (plan_view["joint_trajectory"]) {
        auto joint_traj_doc = plan_view["joint_trajectory"].get_document().value;
        
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
      if (plan_view["multi_dof_joint_trajectory"]) {
        auto multi_dof_doc = plan_view["multi_dof_joint_trajectory"].get_document().value;
        
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
            
            // velocities, accelerations省略
            // ...existing code...
            
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
      
      // Validate trajectory has data
      if (robot_trajectory.joint_trajectory.joint_names.empty() && 
          robot_trajectory.multi_dof_joint_trajectory.joint_names.empty()) {
        RCLCPP_WARN(this->get_logger(), "Record %zu has empty trajectory, skipping", record_idx);
        continue;
      }

      auto& pts = robot_trajectory.joint_trajectory.points;
      if (!pts.empty()) {
        RCLCPP_INFO(this->get_logger(), "[Record %zu TIME BEFORE] tN=%.6f (N=%zu)",
                    record_idx, duration_to_sec(pts.back().time_from_start), pts.size());
      }

      // time_scalingを適用
      if (!scale_trajectory_time(robot_trajectory, time_scale)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to scale trajectory time for record %zu (time_scale=%.3f)", record_idx, time_scale);
        handle_error("Failed to scale trajectory time");
        return;
      }
      
      if (!pts.empty()) {
        RCLCPP_INFO(this->get_logger(), "[Record %zu TIME AFTER ] tN=%.6f",
                    record_idx, duration_to_sec(pts.back().time_from_start));
      }
      
      loaded_plans.push_back(robot_trajectory);
      RCLCPP_INFO(this->get_logger(), "Loaded plan from record %zu with %zu joint trajectory points (time scaled)", 
                  record_idx, robot_trajectory.joint_trajectory.points.size());
    }
    
    if (loaded_plans.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No valid plans were loaded from any record");
      handle_error("No valid plans loaded from database");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu plan(s) from database (all time-scaled)", loaded_plans.size());

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse plans: %s", e.what());
    handle_error("Failed to parse plans from DB");
    return;
  }

  // 複数のtrajectoryを時間を調整して結合
  RCLCPP_INFO(this->get_logger(), "Combining %zu trajectory(ies) by adjusting time_from_start", loaded_plans.size());
    
  moveit_msgs::msg::RobotTrajectory combined_trajectory;
  rclcpp::Duration accumulated_time(0, 0);  // 累積時間

  for (size_t i = 0; i < loaded_plans.size(); ++i) {
    const auto& current_traj = loaded_plans[i];
    
    if (current_traj.joint_trajectory.points.empty()) {
      RCLCPP_WARN(this->get_logger(), "Trajectory %zu is empty, skipping", i+1);
      continue;
    }

    if (i == 0) {
      // 最初のtrajectoryはそのまま使用
      combined_trajectory = current_traj;
      
      // 最後のポイントの時刻を取得
      const auto& last_point = combined_trajectory.joint_trajectory.points.back();
      accumulated_time = last_point.time_from_start;
      
      double duration_sec = accumulated_time.nanoseconds() * 1e-9;
      RCLCPP_INFO(this->get_logger(), "Trajectory 1/%zu: %zu waypoints, duration=%.2f sec", 
                  loaded_plans.size(),
                  combined_trajectory.joint_trajectory.points.size(),
                  duration_sec);
    } else {
      // 2番目以降は時間をオフセットして追加
      // 最初のポイント（前のtrajectoryの最後と重複）はスキップ
      for (size_t j = 1; j < current_traj.joint_trajectory.points.size(); ++j) {
        const auto& point = current_traj.joint_trajectory.points[j];
        auto adjusted_point = point;
        // 累積時間を加算
        adjusted_point.time_from_start = accumulated_time + point.time_from_start;
        combined_trajectory.joint_trajectory.points.push_back(adjusted_point);
      }
      
      // 最後のポイントの時刻を更新
      const auto& last_point = combined_trajectory.joint_trajectory.points.back();
      accumulated_time = last_point.time_from_start;
      
      double duration_sec = accumulated_time.nanoseconds() * 1e-9;
      RCLCPP_INFO(this->get_logger(), "Trajectory %zu/%zu: added %zu waypoints (skipped first), total duration=%.2f sec", 
                  i+1, loaded_plans.size(),
                  current_traj.joint_trajectory.points.size() - 1,
                  duration_sec);
    }
  }

  double total_duration_sec = accumulated_time.nanoseconds() * 1e-9;
  RCLCPP_INFO(this->get_logger(), "Combined trajectory: %zu total waypoints, %.2f sec total duration",
              combined_trajectory.joint_trajectory.points.size(),
              total_duration_sec);

  // 時間が厳密に増加していることを検証
  for (size_t i = 1; i < combined_trajectory.joint_trajectory.points.size(); ++i) {
    const auto& prev_time = combined_trajectory.joint_trajectory.points[i-1].time_from_start;
    const auto& curr_time = combined_trajectory.joint_trajectory.points[i].time_from_start;
    
    // time_from_startを秒単位に変換して比較
    double prev_sec = prev_time.sec + prev_time.nanosec * 1e-9;
    double curr_sec = curr_time.sec + curr_time.nanosec * 1e-9;
    
    if (curr_sec <= prev_sec) {
      RCLCPP_ERROR(this->get_logger(), "Time between points %zu and %zu is not strictly increasing: %.6f and %.6f",
                  i-1, i, prev_sec, curr_sec);
      handle_error("Time is not strictly increasing between waypoints.");
      return;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Time validation passed: all waypoints have strictly increasing time");

  // 結合したtrajectoryを単一のplanとして送信
  goal_msg.plan = combined_trajectory;

  RCLCPP_INFO(this->get_logger(), "Sending EXECUTE_PLAN command with combined trajectory (from %zu original plans) to tms_rp_excavator", 
              loaded_plans.size());

  // traj_follow_recordにも送信
  traj_recorder_msgs::action::TrajFollow::Goal traj_goal;
  traj_goal.plan = goal_msg.plan;
  traj_goal.time_scaling = time_scale;
  traj_goal.velocity_scaling = velocity_scale;
  traj_goal.acceleration_scaling = acceleration_scale;

  auto traj_send_opts = rclcpp_action::Client<traj_recorder_msgs::action::TrajFollow>::SendGoalOptions();
  traj_send_opts.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<traj_recorder_msgs::action::TrajFollow>::SharedPtr& gh)
    {
      if (!gh) {
        RCLCPP_ERROR(this->get_logger(), "traj_follow_record goal rejected");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "traj_follow_record goal accepted");
      traj_goal_handle_ = gh;
    };

  traj_send_opts.feedback_callback =
    [this](auto, const std::shared_ptr<const traj_recorder_msgs::action::TrajFollow::Feedback> fb)
    {
      RCLCPP_INFO(this->get_logger(), "traj_follow_record: %s", fb->status.c_str());
    };

  traj_send_opts.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<traj_recorder_msgs::action::TrajFollow>::WrappedResult& res)
    {
      RCLCPP_INFO(this->get_logger(), "traj_follow_record finished (code=%d)",
                  static_cast<int>(res.code));
      traj_goal_handle_.reset();
    };

  traj_action_client_->async_send_goal(traj_goal, traj_send_opts);

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
  if (traj_goal_handle_) {
    traj_action_client_->async_cancel_goal(traj_goal_handle_);
  }

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
