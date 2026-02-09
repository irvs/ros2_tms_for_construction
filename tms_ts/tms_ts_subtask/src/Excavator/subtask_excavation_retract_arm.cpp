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
  this->declare_parameter<std::string>("planning_group", "manipulator");
  this->declare_parameter<double>("arm_joint_max_limit", 1.57);  // 90度
  this->declare_parameter<double>("search_precision", 0.02);     // 約1.1度
  
  this->get_parameter("planning_group", planning_group_);
  this->get_parameter("arm_joint_max_limit", arm_joint_max_limit_);
  this->get_parameter("search_precision", search_precision_);
  
  RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());
  RCLCPP_INFO(this->get_logger(), "arm_joint_max_limit: %.3f rad (%.1f deg)", 
              arm_joint_max_limit_, arm_joint_max_limit_ * 180.0 / M_PI);
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
      
      // 必要に応じて許容誤差を設定（オプション）
      // ここでは取得した値をログに出力するのみ
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

  // Parse parameters: x, y, z, theta_w from database
  double x, y, z, theta_w;
  try {
    auto doc_x = bsoncxx::from_json(param_from_db_["x"]);
    auto doc_y = bsoncxx::from_json(param_from_db_["y"]);
    auto doc_z = bsoncxx::from_json(param_from_db_["z"]);
    auto doc_theta_w = bsoncxx::from_json(param_from_db_["theta_w"]);

    x = get_numeric_value(doc_x.view()["x"]);
    y = get_numeric_value(doc_y.view()["y"]);
    z = get_numeric_value(doc_z.view()["z"]);
    theta_w = get_numeric_value(doc_theta_w.view()["theta_w"]);
    
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

  // Step 2: 目標姿勢からIK解を取得
  RCLCPP_INFO(this->get_logger(), "Step 2: Planning to target pose...");
  
  auto excavator_goal = ExcavatorAction::Goal();
  excavator_goal.command = ExcavatorAction::Goal::CMD_PLAN_TO_POSE;
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
  RCLCPP_INFO(this->get_logger(), "Original arm_joint angle: %.3f rad (%.1f deg)", 
              original_arm_angle, original_arm_angle * 180.0 / M_PI);

  // Step 3: 2分探索でarm_jointをできるだけ引く
  RCLCPP_INFO(this->get_logger(), "Step 3: Binary search for maximum arm retraction...");
  
  double best_arm_angle;
  tms_msg_rp::msg::TmsRpExcavatorJointValues best_joint_values;
  
  if (!binary_search_max_arm_angle(original_joint_values, original_arm_angle, 
                                    best_arm_angle, best_joint_values)) {
    handle_error("Binary search failed");
    return;
  }

  // Step 4: 元の位置→引いた位置の順に実行
  RCLCPP_INFO(this->get_logger(), "Step 4: Executing motion sequence...");
  
  // 4-1. 元の位置に移動
  RCLCPP_INFO(this->get_logger(), "  4-1. Moving to original position...");
  auto goal_original = ExcavatorAction::Goal();
  goal_original.command = ExcavatorAction::Goal::CMD_PLAN_AND_EXECUTE_JOINTS;
  goal_original.planning_group = planning_group_;
  goal_original.joint_values_sequence.push_back(original_joint_values);

  ExcavatorAction::Result::SharedPtr result_original;
  if (!call_excavator_action_sync(goal_original, result_original)) {
    handle_error("Failed to execute motion to original position");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "  Successfully moved to original position");

  // 4-2. 引いた位置に移動
  RCLCPP_INFO(this->get_logger(), "  4-2. Moving to retracted position...");
  auto goal_retracted = ExcavatorAction::Goal();
  goal_retracted.command = ExcavatorAction::Goal::CMD_PLAN_AND_EXECUTE_JOINTS;
  goal_retracted.planning_group = planning_group_;
  goal_retracted.joint_values_sequence.push_back(best_joint_values);

  ExcavatorAction::Result::SharedPtr result_retracted;
  if (!call_excavator_action_sync(goal_retracted, result_retracted)) {
    handle_error("Failed to execute motion to retracted position");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "  Successfully moved to retracted position");

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
