// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.

#include "tms_ts_subtask/Excavator/subtask_excavation_retract_arm.hpp"
#include "tms_ts_subtask/Excavator/lib/excavator_pose_converter.hpp"
#include <bsoncxx/json.hpp>
#include <cmath>
#include <glog/logging.h>

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
}

SubtaskExcavationRetractArm::SubtaskExcavationRetractArm()
  : SubtaskNodeBase("subtask_excavation_retract_arm"),
    pose_converter_()
{
  this->declare_parameter<double>("search_precision", 0.02);
  this->get_parameter("search_precision", search_precision_);
  RCLCPP_INFO(this->get_logger(), "search_precision: %.3f rad (%.1f deg)", 
              search_precision_, search_precision_ * 180.0 / M_PI);

  // Create action server
  auto options_server = rcl_action_server_get_default_options();
  options_server.goal_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_server.result_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_server.cancel_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_server.feedback_topic_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_server.status_topic_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();

  action_server_ = rclcpp_action::create_server<tms_msg_ts::action::LeafNodeBase>(
      this, "subtask_excavation_retract_arm",
      std::bind(&SubtaskExcavationRetractArm::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SubtaskExcavationRetractArm::handle_cancel, this, std::placeholders::_1),
      std::bind(&SubtaskExcavationRetractArm::handle_accepted, this, std::placeholders::_1),
      options_server);

  // Create action client to tms_rp_excavator
  auto options_client = rcl_action_client_get_default_options();
  options_client.goal_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_client.result_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_client.cancel_service_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_client.feedback_topic_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  options_client.status_topic_qos = rclcpp::QoS(10).reliable().durability_volatile().get_rmw_qos_profile();
  
  action_client_ = rclcpp_action::create_client<ExcavatorAction>(this, "tms_rp_excavator", nullptr, options_client);
  
  if (action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_INFO(this->get_logger(), "Connected to tms_rp_excavator action server");
  } else {
    RCLCPP_ERROR(this->get_logger(), "tms_rp_excavator action server not available");
  }

  // Create service clients for parameter get/set
  param_get_client_ = this->create_client<tms_msg_rp::srv::TmsRpExcavatorParamGet>("tms_rp_excavator_param_get");
  param_set_client_ = this->create_client<tms_msg_rp::srv::TmsRpExcavatorParamSet>("tms_rp_excavator_param_set");
  
  if (param_get_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_INFO(this->get_logger(), "Connected to tms_rp_excavator_param_get service");
  } else {
    RCLCPP_WARN(this->get_logger(), "tms_rp_excavator_param_get service not available yet");
  }
}

rclcpp_action::GoalResponse SubtaskExcavationRetractArm::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request for model: %s, record: %s",
              goal->model_name.c_str(), goal->record_name.c_str());
  
  used_model_name_ = goal->model_name;
  used_record_name_ = goal->record_name;
  param_from_db_ = GetParamFromDBAsJson(goal->model_name, goal->record_name);
  
  if (param_from_db_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameters from DB");
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SubtaskExcavationRetractArm::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  
  if (client_future_goal_handle_.valid() &&
      client_future_goal_handle_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    auto gh = client_future_goal_handle_.get();
    action_client_->async_cancel_goal(gh);
  }
  
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SubtaskExcavationRetractArm::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle)
{
  std::thread{std::bind(&SubtaskExcavationRetractArm::execute, this, std::placeholders::_1),
              goal_handle}.detach();
}

bool SubtaskExcavationRetractArm::call_excavator_action_sync(
    const ExcavatorAction::Goal& goal,
    ExcavatorAction::Result::SharedPtr& result)
{
  auto send_goal_future = action_client_->async_send_goal(goal);
  
  // Wait for the future to complete without spinning the node
  auto status = send_goal_future.wait_for(std::chrono::seconds(30));
  if (status != std::future_status::ready)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send goal (timeout)");
    return false;
  }

  auto goal_handle = send_goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    return false;
  }

  auto result_future = action_client_->async_get_result(goal_handle);
  
  // Wait for the result future to complete without spinning the node
  status = result_future.wait_for(std::chrono::seconds(60));
  if (status != std::future_status::ready)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get result (timeout)");
    return false;
  }

  auto wrapped_result = result_future.get();
  result = wrapped_result.result;
  
  return wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED && result->success;
}

bool SubtaskExcavationRetractArm::binary_search_max_arm_angle(
    const tms_msg_rp::msg::TmsRpExcavatorJointValues& original_joint_values,
    double original_arm_angle,
    double& best_arm_angle,
    tms_msg_rp::msg::TmsRpExcavatorJointValues& best_joint_values)
{
  // arm_jointのインデックスを見つける
  int arm_joint_idx = -1;
  for (size_t i = 0; i < original_joint_values.joint_names.size(); ++i) {
    if (original_joint_values.joint_names[i] == "arm_joint") {
      arm_joint_idx = i;
      break;
    }
  }

  if (arm_joint_idx == -1) {
    RCLCPP_ERROR(this->get_logger(), "arm_joint not found in joint_names");
    return false;
  }

  double search_min = original_arm_angle;
  double search_max = arm_joint_max_limit_;
  best_arm_angle = original_arm_angle;
  best_joint_values = original_joint_values;

  RCLCPP_INFO(this->get_logger(), "Starting binary search for arm_joint angle");
  RCLCPP_INFO(this->get_logger(), "  Range: %.3f to %.3f rad (%.1f to %.1f deg)", 
              search_min, search_max, search_min * 180.0 / M_PI, search_max * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "  Maximum possible retraction: %.3f rad (%.1f deg)",
              search_max - search_min, (search_max - search_min) * 180.0 / M_PI);

  int iteration = 0;
  while (search_max - search_min > search_precision_) {
    double mid = (search_min + search_max) / 2.0;
    iteration++;

    RCLCPP_INFO(this->get_logger(), "Iteration %d: Testing arm_joint angle %.3f rad (%.1f deg)", 
                iteration, mid, mid * 180.0 / M_PI);

    // arm_jointの角度を変更した目標関節値を作成
    tms_msg_rp::msg::TmsRpExcavatorJointValues test_joint_values = original_joint_values;
    test_joint_values.joint_values[arm_joint_idx] = mid;

    // CMD_PLAN_TO_JOINTSでプランニングを試す
    auto excavator_goal = ExcavatorAction::Goal();
    excavator_goal.command = ExcavatorAction::Goal::CMD_PLAN_TO_JOINTS;
    excavator_goal.planning_group = planning_group_;
    excavator_goal.joint_values_sequence.push_back(test_joint_values);

    ExcavatorAction::Result::SharedPtr result;
    bool success = call_excavator_action_sync(excavator_goal, result);

    if (success) {
      // プランが成功した場合、さらにアームを引けるか試す
      best_arm_angle = mid;
      best_joint_values = test_joint_values;
      search_min = mid;
      
      RCLCPP_INFO(this->get_logger(), "  -> Planning succeeded! Retracted by %.3f rad (%.1f deg)", 
                  mid - original_arm_angle, (mid - original_arm_angle) * 180.0 / M_PI);
    } else {
      // プランが失敗した場合、引きすぎなので範囲を狭める
      search_max = mid;
      RCLCPP_DEBUG(this->get_logger(), "  -> Planning failed");
    }
  }

  RCLCPP_INFO(this->get_logger(), "Binary search completed in %d iterations", iteration);
  RCLCPP_INFO(this->get_logger(), "Best arm_joint angle: %.3f rad (%.1f deg)", 
              best_arm_angle, best_arm_angle * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "Arm retraction: %.3f rad (%.1f deg)", 
              best_arm_angle - original_arm_angle, 
              (best_arm_angle - original_arm_angle) * 180.0 / M_PI);

  return true;
}

void SubtaskExcavationRetractArm::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto result = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();

  // Function for error handling
  auto handle_error = [&](const std::string& message) {
    if (goal_handle->is_active()) {
      result->result = false;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal is not active");
    }
  };

  if (!action_client_->action_server_is_ready()) {
    handle_error("Action server not available");
    return;
  }

  auto doc = bsoncxx::from_json(param_from_db_["planning_group"]);
  auto view = doc.view();
  if (!view["planning_group"] || view["planning_group"].type() != bsoncxx::type::k_string) {
    handle_error("planning_group must be an string type");
    return;
  }
  planning_group_ = view["planning_group"].get_string().value.to_string();

  // Step 0: サービスを使って関節制限と許容誤差を取得
  RCLCPP_INFO(this->get_logger(), "Step 0: Getting joint limits and goal tolerances from parameter service...");
  
  auto param_request = std::make_shared<tms_msg_rp::srv::TmsRpExcavatorParamGet::Request>();
  param_request->get_joint_limits = true;
  param_request->get_current_state = false;
  param_request->get_configuration = true;
  
  auto param_future = param_get_client_->async_send_request(param_request);
  
  // サービスコールの完了を待つ
  auto status = param_future.wait_for(std::chrono::seconds(10));
  if (status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "Failed to get parameters from service (timeout), using default values");
  } else {
    auto param_response = param_future.get();
    
    if (param_response->success) {
      // arm_jointのインデックスを見つけて最大値を取得
      int arm_joint_idx = -1;
      for (size_t i = 0; i < param_response->joint_names.size(); ++i) {
        if (param_response->joint_names[i] == "arm_joint") {
          arm_joint_idx = i;
          break;
        }
      }
      
      if (arm_joint_idx != -1 && arm_joint_idx < static_cast<int>(param_response->max_positions.size())) {
        arm_joint_max_limit_ = param_response->max_positions[arm_joint_idx];
        RCLCPP_INFO(this->get_logger(), "Got arm_joint max limit from service: %.3f rad (%.1f deg)",
                    arm_joint_max_limit_, arm_joint_max_limit_ * 180.0 / M_PI);
      } else {
        RCLCPP_WARN(this->get_logger(), "arm_joint not found in joint limits, using default: %.3f rad",
                    arm_joint_max_limit_);
      }
      
      // 許容誤差も取得
      RCLCPP_INFO(this->get_logger(), "Current goal tolerances:");
      RCLCPP_INFO(this->get_logger(), "  Position: %.4f m", param_response->goal_position_tolerance);
      RCLCPP_INFO(this->get_logger(), "  Orientation: %.4f rad (%.2f deg)", 
                  param_response->goal_orientation_tolerance,
                  param_response->goal_orientation_tolerance * 180.0 / M_PI);
      RCLCPP_INFO(this->get_logger(), "  Joint: %.4f rad (%.2f deg)", 
                  param_response->goal_joint_tolerance,
                  param_response->goal_joint_tolerance * 180.0 / M_PI);
      
      // Tolerance設定（必要であれば変更）
      // 例: より厳しい許容誤差に設定する場合
      auto param_set_request = std::make_shared<tms_msg_rp::srv::TmsRpExcavatorParamSet::Request>();
      param_set_request->goal_position_tolerance = 0.1; 
      param_set_request->goal_orientation_tolerance = 0.1; 
      // param_set_request->goal_joint_tolerance = 0.1; 
      
      auto param_set_future = param_set_client_->async_send_request(param_set_request);
      auto set_status = param_set_future.wait_for(std::chrono::seconds(5));
      
      if (set_status == std::future_status::ready) {
        auto param_set_response = param_set_future.get();
        if (param_set_response->success) {
          RCLCPP_INFO(this->get_logger(), "Updated goal tolerances:");
          RCLCPP_INFO(this->get_logger(), "  Position: %.4f m", param_set_response->goal_position_tolerance);
          RCLCPP_INFO(this->get_logger(), "  Orientation: %.4f rad (%.2f deg)", 
                      param_set_response->goal_orientation_tolerance,
                      param_set_response->goal_orientation_tolerance * 180.0 / M_PI);
          RCLCPP_INFO(this->get_logger(), "  Joint: %.4f rad (%.2f deg)", 
                      param_set_response->goal_joint_tolerance,
                      param_set_response->goal_joint_tolerance * 180.0 / M_PI);
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to set tolerances: %s", param_set_response->message.c_str());
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Timeout setting tolerances");
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Parameter service returned failure: %s", 
                  param_response->message.c_str());
    }
  }

  RCLCPP_INFO(this->get_logger(), "Getting parameters from DB");
  RCLCPP_INFO(this->get_logger(), "param_from_db_ contents:");
  for (const auto& [key, value] : param_from_db_) {
    RCLCPP_INFO(this->get_logger(), "  %s: %s", key.c_str(), value.c_str());
  }

  // waypoints形式のみサポート
  if (!param_from_db_.count("waypoints")) {
    handle_error("waypoints field not found in DB. Please use waypoints format.");
    return;
  }

  // Parse parameters: x, y, z, theta_w from database
  double x, y, z, theta_w;
  try {
    auto doc = bsoncxx::from_json(param_from_db_["waypoints"]);
    auto view = doc.view();
    auto waypoints_array = view["waypoints"].get_array().value;
    auto waypoint_element = *waypoints_array.begin();
    auto waypoint_doc = waypoint_element.get_document().value;
    
    if (!view["waypoints"] || view["waypoints"].type() != bsoncxx::type::k_array) {
      handle_error("waypoints must be an array");
      return;
    }
    
    size_t num_waypoints = std::distance(waypoints_array.begin(), waypoints_array.end());
    
    if (num_waypoints != 1) {
      handle_error("waypoints array is expected to contain exactly one waypoint for this subtask");
      return;
    }

    if (!waypoint_doc["data"]) {
      handle_error("Waypoint missing 'data' field");
      return;
    }
    
    auto data_doc = waypoint_doc["data"].get_document().value;
    
    if (!data_doc["x"] || !data_doc["y"] || !data_doc["z"] || !data_doc["theta_w"]) {
      handle_error("Pose waypoint missing required fields (x, y, z, theta_w)");
      return;
    }
    
    x = get_numeric_value(data_doc["x"]);
    y = get_numeric_value(data_doc["y"]);
    z = get_numeric_value(data_doc["z"]);
    theta_w = get_numeric_value(data_doc["theta_w"]);
    
    RCLCPP_INFO(this->get_logger(), "Target: x=%.3f, y=%.3f, z=%.3f, theta_w=%.3f",
                x, y, z, theta_w);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse parameters: %s", e.what());
    handle_error("Failed to parse parameters from DB");
    return;
  }

  // Step 1: pose_converterを使ってxyz, theta_wを7つの変数（xyz + quaternion）に変換
  RCLCPP_INFO(this->get_logger(), "Step 1: Converting pose using pose_converter...");
  
  Pose target_pose;
  pose_converter_.convertToXYZQuaternion(x, y, z, theta_w, target_pose);

  RCLCPP_INFO(this->get_logger(), "Converted to pose: [%.3f, %.3f, %.3f] quat: [%.3f, %.3f, %.3f, %.3f]", 
              target_pose.x, target_pose.y, target_pose.z, 
              target_pose.qx, target_pose.qy, target_pose.qz, target_pose.qw);

  // Step 2: 目標姿勢のIK解を計算（実行はしない）
  RCLCPP_INFO(this->get_logger(), "Step 2: Planning to target pose to get IK solution...");
  
  auto excavator_goal = ExcavatorAction::Goal();
  excavator_goal.command = ExcavatorAction::Goal::CMD_PLAN_TO_POSE;  // プランのみ
  excavator_goal.planning_group = planning_group_;

  geometry_msgs::msg::Pose target_pose_msg;
  target_pose_msg.position.x = target_pose.x;
  target_pose_msg.position.y = target_pose.y;
  target_pose_msg.position.z = target_pose.z;
  target_pose_msg.orientation.x = target_pose.qx;
  target_pose_msg.orientation.y = target_pose.qy;
  target_pose_msg.orientation.z = target_pose.qz;
  target_pose_msg.orientation.w = target_pose.qw;

  excavator_goal.pose_sequence.push_back(target_pose_msg);

  ExcavatorAction::Result::SharedPtr plan_result;
  if (!call_excavator_action_sync(excavator_goal, plan_result)) {
    handle_error("Failed to plan to target pose");
    return;
  }

  if (plan_result->plan.empty()) {
    handle_error("Planning result contains no trajectory");
    return;
  }

  // プランから最終的な関節値を取得
  const auto& trajectory = plan_result->plan[0].joint_trajectory;
  if (trajectory.points.empty()) {
    handle_error("Planned trajectory is empty");
    return;
  }

  // 最後のウェイポイントから関節値を取得
  const auto& last_point = trajectory.points.back();
  
  tms_msg_rp::msg::TmsRpExcavatorJointValues original_joint_values;
  original_joint_values.joint_names = trajectory.joint_names;
  original_joint_values.joint_values.assign(last_point.positions.begin(), last_point.positions.end());

  RCLCPP_INFO(this->get_logger(), "Target pose joint values obtained:");
  for (size_t i = 0; i < original_joint_values.joint_names.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "  %s: %.3f rad (%.1f deg)",
                original_joint_values.joint_names[i].c_str(),
                original_joint_values.joint_values[i],
                original_joint_values.joint_values[i] * 180.0 / M_PI);
  }

  // arm_jointの元の角度を取得
  int arm_joint_idx = -1;
  for (size_t i = 0; i < original_joint_values.joint_names.size(); ++i) {
    if (original_joint_values.joint_names[i] == "arm_joint") {
      arm_joint_idx = i;
      break;
    }
  }

  if (arm_joint_idx == -1) {
    handle_error("arm_joint not found in trajectory");
    return;
  }

  double original_arm_angle = original_joint_values.joint_values[arm_joint_idx];
  RCLCPP_INFO(this->get_logger(), "Target arm_joint angle: %.3f rad (%.1f deg)", 
              original_arm_angle, original_arm_angle * 180.0 / M_PI);

  // Step 3: 2分探索でarm_jointをできるだけ引く
  RCLCPP_INFO(this->get_logger(), "Step 3: Binary search for maximum arm retraction...");
  
  // Step 3-0: 現在のプランナーを取得してPTPに切り替え
  RCLCPP_INFO(this->get_logger(), "  Switching to Pilz PTP planner for binary search...");
  
  auto param_get_planner = std::make_shared<tms_msg_rp::srv::TmsRpExcavatorParamGet::Request>();
  param_get_planner->get_joint_limits = false;
  param_get_planner->get_current_state = false;
  param_get_planner->get_configuration = true;
  
  auto param_get_planner_future = param_get_client_->async_send_request(param_get_planner);
  auto get_planner_status = param_get_planner_future.wait_for(std::chrono::seconds(5));
  
  std::string original_planner_id = "";
  std::string original_pipeline_id = "";
  if (get_planner_status == std::future_status::ready) {
    auto param_get_planner_response = param_get_planner_future.get();
    if (param_get_planner_response->success) {
      original_planner_id = param_get_planner_response->planner_id;
      original_pipeline_id = param_get_planner_response->planning_pipeline_id;
      RCLCPP_INFO(this->get_logger(), "  Current planner: %s, pipeline: %s", 
                  original_planner_id.empty() ? "(default)" : original_planner_id.c_str(),
                  original_pipeline_id.empty() ? "(default)" : original_pipeline_id.c_str());
    }
  }
  
  // Pilz パイプラインとPTPプランナーに切り替え
  auto param_set_ptp = std::make_shared<tms_msg_rp::srv::TmsRpExcavatorParamSet::Request>();
  param_set_ptp->goal_position_tolerance = -1.0;  // 変更しない
  param_set_ptp->goal_orientation_tolerance = -1.0;
  param_set_ptp->goal_joint_tolerance = -1.0;
  param_set_ptp->max_velocity_scaling_factor = -1.0;
  param_set_ptp->max_acceleration_scaling_factor = -1.0;
  param_set_ptp->planning_time = -1.0;
  param_set_ptp->num_planning_attempts = -1;
  param_set_ptp->allow_replanning = false;
  param_set_ptp->planner_id = "PTP";
  param_set_ptp->planning_pipeline_id = "pilz_industrial_motion_planner";
  
  auto param_set_ptp_future = param_set_client_->async_send_request(param_set_ptp);
  auto set_ptp_status = param_set_ptp_future.wait_for(std::chrono::seconds(5));
  
  if (set_ptp_status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "  Failed to switch to Pilz PTP planner, continuing with current planner");
  } else {
    auto param_set_ptp_response = param_set_ptp_future.get();
    if (param_set_ptp_response->success) {
      RCLCPP_INFO(this->get_logger(), "  Switched to Pilz PTP planner");
    } else {
      RCLCPP_WARN(this->get_logger(), "  Failed to switch to Pilz PTP planner: %s", 
                  param_set_ptp_response->message.c_str());
    }
  }
  
  double best_arm_angle;
  tms_msg_rp::msg::TmsRpExcavatorJointValues best_joint_values;
  
  if (!binary_search_max_arm_angle(original_joint_values, original_arm_angle, 
                                    best_arm_angle, best_joint_values)) {
    handle_error("Binary search failed");
    return;
  }
  
  // Step 3-1: 元のプランナーとパイプラインに戻す
  RCLCPP_INFO(this->get_logger(), "  Restoring original planner and pipeline...");
  
  auto param_restore = std::make_shared<tms_msg_rp::srv::TmsRpExcavatorParamSet::Request>();
  param_restore->goal_position_tolerance = -1.0;
  param_restore->goal_orientation_tolerance = -1.0;
  param_restore->goal_joint_tolerance = -1.0;
  param_restore->max_velocity_scaling_factor = -1.0;
  param_restore->max_acceleration_scaling_factor = -1.0;
  param_restore->planning_time = -1.0;
  param_restore->num_planning_attempts = -1;
  param_restore->allow_replanning = false;
  param_restore->planner_id = original_planner_id.empty() ? "" : original_planner_id;
  param_restore->planning_pipeline_id = original_pipeline_id.empty() ? "" : original_pipeline_id;
  
  auto param_restore_future = param_set_client_->async_send_request(param_restore);
  auto restore_status = param_restore_future.wait_for(std::chrono::seconds(5));
  
  if (restore_status == std::future_status::ready) {
    auto param_restore_response = param_restore_future.get();
    if (param_restore_response->success) {
      RCLCPP_INFO(this->get_logger(), "  Restored to original planner: %s, pipeline: %s",
                  param_restore_response->planner_id.c_str(),
                  param_restore_response->planning_pipeline_id.c_str());
    }
  }

  // Step 4: 現在位置から 元の位置→引いた位置→バケット水平 の順に実行（滑らかな軌道で）
  RCLCPP_INFO(this->get_logger(), "Step 4: Executing smooth motion sequence with bucket adjustment...");
  
  // Step 4-0: 現在の関節状態を取得してstart_stateに設定
  RCLCPP_INFO(this->get_logger(), "  Getting current joint state for start_state...");
  
  auto param_request_state = std::make_shared<tms_msg_rp::srv::TmsRpExcavatorParamGet::Request>();
  param_request_state->get_joint_limits = false;
  param_request_state->get_current_state = true;
  param_request_state->get_configuration = false;
  
  auto param_future_state = param_get_client_->async_send_request(param_request_state);
  auto status_state = param_future_state.wait_for(std::chrono::seconds(10));
  
  if (status_state != std::future_status::ready) {
    handle_error("Failed to get current joint state (timeout)");
    return;
  }
  
  auto param_response_state = param_future_state.get();
  if (!param_response_state->success) {
    handle_error("Failed to get current joint state: " + param_response_state->message);
    return;
  }
  
  tms_msg_rp::msg::TmsRpExcavatorJointValues current_joint_values;
  current_joint_values.joint_names = param_response_state->joint_names;
  current_joint_values.joint_values = param_response_state->joint_positions;

  RCLCPP_INFO(this->get_logger(), "  Current joint state:");
  for (size_t i = 0; i < current_joint_values.joint_names.size(); ++i) {
    RCLCPP_INFO(this->get_logger(), "    %s: %.3f rad (%.1f deg)",
                current_joint_values.joint_names[i].c_str(),
                current_joint_values.joint_values[i],
                current_joint_values.joint_values[i] * 180.0 / M_PI);
  }
  
  // Step 4-1: バケット水平位置を計算
  int boom_idx = -1, arm_idx = -1, bucket_idx = -1;
  for (size_t i = 0; i < best_joint_values.joint_names.size(); ++i) {
    if (best_joint_values.joint_names[i] == "boom_joint") boom_idx = i;
    else if (best_joint_values.joint_names[i] == "arm_joint") arm_idx = i;
    else if (best_joint_values.joint_names[i] == "bucket_joint") bucket_idx = i;
  }
  
  if (boom_idx == -1 || arm_idx == -1 || bucket_idx == -1) {
    handle_error("Could not find boom/arm/bucket joints");
    return;
  }
  
  // 引いた位置でのバケット水平角度を計算
  double retracted_boom_angle = best_joint_values.joint_values[boom_idx];
  double retracted_arm_angle = best_joint_values.joint_values[arm_idx];
  double target_bucket_angle = -(retracted_boom_angle + retracted_arm_angle) + M_PI;
  
  RCLCPP_INFO(this->get_logger(), "  Calculating bucket horizontal angle:");
  RCLCPP_INFO(this->get_logger(), "    Retracted boom:   %.3f rad (%.1f deg)", 
              retracted_boom_angle, retracted_boom_angle * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "    Retracted arm:    %.3f rad (%.1f deg)", 
              retracted_arm_angle, retracted_arm_angle * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "    Target bucket:    %.3f rad (%.1f deg)",
              target_bucket_angle, target_bucket_angle * 180.0 / M_PI);
  
  // バケット水平位置の関節値を作成
  tms_msg_rp::msg::TmsRpExcavatorJointValues horizontal_joint_values = best_joint_values;
  horizontal_joint_values.joint_values[bucket_idx] = target_bucket_angle;
  
  // MotionSequenceItemを作成（3点：目標姿勢→引いた位置→バケット水平）
  std::vector<moveit_msgs::msg::MotionSequenceItem> motion_sequence_items;
  
  // 4-2. 目標姿勢へのMotionSequenceItem
  moveit_msgs::msg::MotionSequenceItem item1;
  item1.req.group_name = planning_group_;
  item1.req.max_velocity_scaling_factor = 1.0;
  item1.req.max_acceleration_scaling_factor = 1.0;
  item1.req.allowed_planning_time = 5.0;
  item1.req.planner_id = "PTP";
  item1.req.pipeline_id = "pilz_industrial_motion_planner";
  
  // start_stateを設定（現在の関節状態）
  sensor_msgs::msg::JointState start_state;
  start_state.name = current_joint_values.joint_names;
  start_state.position = current_joint_values.joint_values;
  item1.req.start_state.joint_state = start_state;
  item1.req.start_state.is_diff = false;
  
  // Joint constraintとして目標を設定
  moveit_msgs::msg::Constraints constraints1;
  for (size_t i = 0; i < original_joint_values.joint_names.size(); ++i) {
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = original_joint_values.joint_names[i];
    joint_constraint.position = original_joint_values.joint_values[i];
    joint_constraint.tolerance_above = 0.01;
    joint_constraint.tolerance_below = 0.01;
    joint_constraint.weight = 1.0;
    constraints1.joint_constraints.push_back(joint_constraint);
  }
  item1.req.goal_constraints.push_back(constraints1);
  
  // ブレンド（滑らかに次へ）
  item1.blend_radius = 0.001;
  
  RCLCPP_INFO(this->get_logger(), "  Item 1: Target position with blend_radius=%.3f rad (%.1f deg)",
              item1.blend_radius, item1.blend_radius * 180.0 / M_PI);
  
  motion_sequence_items.push_back(item1);
  
  // 4-3. 引いた位置へのMotionSequenceItem
  moveit_msgs::msg::MotionSequenceItem item2;
  item2.req.group_name = planning_group_;
  item2.req.max_velocity_scaling_factor = 1.0;
  item2.req.max_acceleration_scaling_factor = 1.0;
  item2.req.allowed_planning_time = 5.0;
  item2.req.planner_id = "PTP";
  item2.req.pipeline_id = "pilz_industrial_motion_planner";
  
  // Joint constraintとして目標を設定
  moveit_msgs::msg::Constraints constraints2;
  for (size_t i = 0; i < best_joint_values.joint_names.size(); ++i) {
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = best_joint_values.joint_names[i];
    joint_constraint.position = best_joint_values.joint_values[i];
    joint_constraint.tolerance_above = 0.01;
    joint_constraint.tolerance_below = 0.01;
    joint_constraint.weight = 1.0;
    constraints2.joint_constraints.push_back(joint_constraint);
  }
  item2.req.goal_constraints.push_back(constraints2);
  
  // ブレンド（滑らかに次へ）
  item2.blend_radius = 0.001;
  
  RCLCPP_INFO(this->get_logger(), "  Item 2: Retracted position with blend_radius=%.3f rad (%.1f deg)",
              item2.blend_radius, item2.blend_radius * 180.0 / M_PI);
  
  motion_sequence_items.push_back(item2);
  
  // 4-4. バケット水平位置へのMotionSequenceItem
  moveit_msgs::msg::MotionSequenceItem item3;
  item3.req.group_name = planning_group_;
  item3.req.max_velocity_scaling_factor = 1.0;
  item3.req.max_acceleration_scaling_factor = 1.0;
  item3.req.allowed_planning_time = 5.0;
  item3.req.planner_id = "PTP";
  item3.req.pipeline_id = "pilz_industrial_motion_planner";
  
  // Joint constraintとして目標を設定
  moveit_msgs::msg::Constraints constraints3;
  for (size_t i = 0; i < horizontal_joint_values.joint_names.size(); ++i) {
    moveit_msgs::msg::JointConstraint joint_constraint;
    joint_constraint.joint_name = horizontal_joint_values.joint_names[i];
    joint_constraint.position = horizontal_joint_values.joint_values[i];
    joint_constraint.tolerance_above = 0.01;
    joint_constraint.tolerance_below = 0.01;
    joint_constraint.weight = 1.0;
    constraints3.joint_constraints.push_back(joint_constraint);
  }
  item3.req.goal_constraints.push_back(constraints3);
  
  // 最後のウェイポイントはblend_radius=0（完全に停止）
  item3.blend_radius = 0.0;
  
  RCLCPP_INFO(this->get_logger(), "  Item 3: Bucket horizontal position with blend_radius=%.3f (stop)",
              item3.blend_radius);
  RCLCPP_INFO(this->get_logger(), "    Bucket angle change: %.3f rad (%.1f deg)",
              target_bucket_angle - best_joint_values.joint_values[bucket_idx],
              (target_bucket_angle - best_joint_values.joint_values[bucket_idx]) * 180.0 / M_PI);
  
  motion_sequence_items.push_back(item3);
  
  // CMD_PLAN_AND_EXECUTE_MOTION_SEQUENCEで実行
  auto goal_sequence = ExcavatorAction::Goal();
  goal_sequence.command = ExcavatorAction::Goal::CMD_PLAN_AND_EXECUTE_MOTION_SEQUENCE;
  goal_sequence.planning_group = planning_group_;
  goal_sequence.motion_sequence_items = motion_sequence_items;
  
  RCLCPP_INFO(this->get_logger(), "  Sending motion sequence with %zu items (start from current state)...", 
              motion_sequence_items.size());
  
  ExcavatorAction::Result::SharedPtr result_sequence;
  if (!call_excavator_action_sync(goal_sequence, result_sequence)) {
    handle_error("Failed to execute smooth motion sequence");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "  Successfully executed smooth motion sequence!");
  RCLCPP_INFO(this->get_logger(), "  Total arm retraction: %.3f rad (%.1f deg)",
              best_arm_angle - original_arm_angle,
              (best_arm_angle - original_arm_angle) * 180.0 / M_PI);
  RCLCPP_INFO(this->get_logger(), "  Bucket adjusted to horizontal position");

  // 成功
  result->result = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Arm retraction subtask completed successfully!");
}

int main(int argc, char* argv[])
{
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SubtaskExcavationRetractArm>());
  rclcpp::shutdown();
  return 0;
}
