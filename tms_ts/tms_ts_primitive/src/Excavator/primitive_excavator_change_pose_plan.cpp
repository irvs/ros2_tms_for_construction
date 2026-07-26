// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
// Licensed under the Apache License, Version 2.0

#include "tms_ts_primitive/Excavator/primitive_excavator_change_pose_plan.hpp"

using namespace std::chrono_literals;

// Helper function to get numeric value from BSON element
namespace {
  double get_numeric_value(const bsoncxx::document::element& element) {
    switch (element.type()) {
      RCLCPP_INFO(rclcpp::get_logger("PrimitiveExcavatorChangePosePlan"), "Element type: %d", static_cast<int>(element.type()));
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
}

PrimitiveExcavatorChangePosePlan::PrimitiveExcavatorChangePosePlan() 
  : PrimitiveNodeBase("primitive_excavator_change_pose_plan_node")
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
      this, "primitive_excavator_change_pose_plan",
      std::bind(&PrimitiveExcavatorChangePosePlan::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PrimitiveExcavatorChangePosePlan::handle_cancel, this, std::placeholders::_1),
      std::bind(&PrimitiveExcavatorChangePosePlan::handle_accepted, this, std::placeholders::_1),
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

  param_get_client_ = this->create_client<tms_msg_rp::srv::TmsRpExcavatorParamGet>("tms_rp_excavator_param_get");
  param_set_client_ = this->create_client<tms_msg_rp::srv::TmsRpExcavatorParamSet>("tms_rp_excavator_param_set");
  
  if (param_get_client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_INFO(this->get_logger(), "Connected to tms_rp_excavator_param_get service");
  } else {
    RCLCPP_WARN(this->get_logger(), "tms_rp_excavator_param_get service not available yet");
  }

}

rclcpp_action::GoalResponse PrimitiveExcavatorChangePosePlan::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
{
  used_model_name_ = goal->model_name;
  used_record_name_ = goal->record_name;
  previous_target_record_name_ = goal->previous_target_record_name;
  
  param_from_db_ = GetParamFromDBAsJson(used_model_name_, used_record_name_);
  if (param_from_db_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameters from DB");
    return rclcpp_action::GoalResponse::REJECT;
  }
  
  if (!previous_target_record_name_.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Previous plan record: %s", previous_target_record_name_.c_str());
    previous_param_from_db_ = GetParamFromDBAsJson(used_model_name_, previous_target_record_name_);
    if (previous_param_from_db_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get parameters from DB");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PrimitiveExcavatorChangePosePlan::handle_cancel(
    const std::shared_ptr<GoalHandle> goal_handle)
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

void PrimitiveExcavatorChangePosePlan::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  using namespace std::placeholders;
  std::thread{ std::bind(&PrimitiveExcavatorChangePosePlan::execute, this, _1), goal_handle }.detach();
}

void PrimitiveExcavatorChangePosePlan::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto result = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();
  auto goal_msg = TmsRpExcavator::Goal();

  // エラー処理用のラムダ関数
  auto handle_error = [&](const std::string& message) {
    if (goal_handle->is_active())
    {
      result->result = false;
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), "%s", message.c_str());
    }
  };
  
  // 準備
  if (!action_client_->action_server_is_ready()) // Action serverの準備ができているか確認
  {
    handle_error("Action server not available");
    return;
  }
  if (!parse_previous_plan(goal_msg)) // Parse previous plan if specified
  {
    handle_error("Failed to parse previous plan");
    return;
  }
  if (!param_from_db_.count("waypoints")) { // waypoints形式のみサポート
    handle_error("waypoints field not found in DB. Please use waypoints format.");
    return;
  }

  // plannerを指定
  tms_msg_rp::srv::TmsRpExcavatorParamSet::Response::SharedPtr set_param_response;
  auto set_param_request = std::make_shared<tms_msg_rp::srv::TmsRpExcavatorParamSet::Request>();
  set_param_request->planning_pipeline_id = "ompl";
  set_param_request->planner_id = "RRTConnectkConfigDefault";
  auto set_param_future = param_set_client_->async_send_request(set_param_request);
  auto set_status = set_param_future.wait_for(std::chrono::seconds(5));
  if (set_status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "Failed to set planner parameters (timeout)");
  } else {
    set_param_response = set_param_future.get();
    if (!set_param_response->success) {
      RCLCPP_WARN(this->get_logger(), "Failed to set planner parameters: %s", set_param_response->message.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Planner parameters set successfully");
    }
  }

  // データベースから速度・加速度スケーリングを取得して反映
  auto it_vel = param_from_db_.find("velocity_scale");
  if (it_vel != param_from_db_.end()) {
    auto doc_velocity_scaling = bsoncxx::from_json(it_vel->second);
    auto view_velocity_scaling = doc_velocity_scaling.view();
    double velocity_scaling =
        get_numeric_value(view_velocity_scaling["velocity_scale"]);

    if (velocity_scaling <= 0.0 || velocity_scaling > 1.0) {
      RCLCPP_ERROR(this->get_logger(),
                  "Invalid velocity_scale from DB: %.3f. Must be in (0, 1].",
                  velocity_scaling);
      return;
    }
    velocity_scaling_ = velocity_scaling;
  }
  auto it_acc = param_from_db_.find("acceleration_scale");
  if (it_acc != param_from_db_.end()) {
    auto doc_acceleration_scaling = bsoncxx::from_json(it_acc->second);
    auto view_acceleration_scaling = doc_acceleration_scaling.view();
    double acceleration_scaling =
        get_numeric_value(view_acceleration_scaling["acceleration_scale"]);

    if (acceleration_scaling <= 0.0 || acceleration_scaling > 1.0) {
      RCLCPP_ERROR(this->get_logger(),
                  "Invalid acceleration_scale from DB: %.3f. Must be in (0, 1].",
                  acceleration_scaling);
      return;
    }
    acceleration_scaling_ = acceleration_scaling;
  }
  // toleranceと速度を設定
  auto param_request_config = std::make_shared<tms_msg_rp::srv::TmsRpExcavatorParamGet::Request>();
  param_request_config->get_joint_limits = false;
  param_request_config->get_current_state = false;
  param_request_config->get_configuration = true;
  auto param_future_config = param_get_client_->async_send_request(param_request_config);
  auto status_config = param_future_config.wait_for(std::chrono::seconds(10));
  if (status_config != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "Failed to get parameters from service (timeout), using default values");
  } else {
    auto param_response = param_future_config.get();
    if (param_response->success) {
      RCLCPP_INFO(this->get_logger(), "Current goal tolerances:");
      RCLCPP_INFO(this->get_logger(), "  Position: %.4f m", param_response->goal_position_tolerance);
      RCLCPP_INFO(this->get_logger(), "  Orientation: %.4f rad (%.2f deg)", 
                  param_response->goal_orientation_tolerance,
                  param_response->goal_orientation_tolerance * 180.0 / M_PI);
      RCLCPP_INFO(this->get_logger(), "  Joint: %.4f rad (%.2f deg)", 
                  param_response->goal_joint_tolerance,
                  param_response->goal_joint_tolerance * 180.0 / M_PI);

      auto param_set_request = std::make_shared<tms_msg_rp::srv::TmsRpExcavatorParamSet::Request>();
      param_set_request->goal_position_tolerance = 0.5; 
      param_set_request->goal_orientation_tolerance = 0.5; 

      param_set_request->max_velocity_scaling_factor = velocity_scaling_;
      param_set_request->max_acceleration_scaling_factor = acceleration_scaling_;
      
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


  tms_msg_rp::srv::TmsRpExcavatorParamGet::Response::SharedPtr param_response;

  // サービスからcurrent_statesのみ取得して保持
  auto param_request = std::make_shared<tms_msg_rp::srv::TmsRpExcavatorParamGet::Request>();
  param_request->get_joint_limits = true;
  param_request->get_current_state = true;
  param_request->get_configuration = false;
  auto param_future = param_get_client_->async_send_request(param_request);  
  auto status = param_future.wait_for(std::chrono::seconds(10));
  if (status != std::future_status::ready) {
    RCLCPP_WARN(this->get_logger(), "Failed to get parameters from service (timeout), using default values");
  } else {
    param_response = param_future.get();
    if (param_response->success) {
      current_joint_states_ = param_response->joint_states;
      // joint_limitsについては、param_response->joint_limits を参照
      RCLCPP_INFO(this->get_logger(), "Retrieved current joint states for %zu joints", current_joint_states_.name.size());
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to get current joint states: %s", param_response->message.c_str());
    }
  }

  // ゴールメッセージの構築
  auto doc = bsoncxx::from_json(param_from_db_["planning_group"]);
  auto view = doc.view();
  if (!view["planning_group"] || view["planning_group"].type() != bsoncxx::type::k_string) {
    handle_error("planning_group must be an string type");
    return;
  }
  planning_group_ = view["planning_group"].get_string().value.to_string();
  goal_msg.planning_group = planning_group_;
  RCLCPP_INFO(this->get_logger(), "Parsing waypoints for planning");

  try {
    auto doc = bsoncxx::from_json(param_from_db_["waypoints"]);
    auto view = doc.view();    
    if (!view["waypoints"] || view["waypoints"].type() != bsoncxx::type::k_array) {
      handle_error("waypoints must be an array");
      return;
    }
    auto waypoints_array = view["waypoints"].get_array().value;
    size_t num_waypoints = std::distance(waypoints_array.begin(), waypoints_array.end());
    if (num_waypoints == 0) {
      handle_error("waypoints array is empty");
      return;
    }

    std::unordered_map<int, trajectory_msgs::msg::JointTrajectoryPoint> start_cache;
    start_cache.clear();
    
    // waypointsの数で処理を分岐
    if (num_waypoints == 1) {
      // ========== 1個の場合: CMD_PLAN_TO_JOINTS または CMD_PLAN_TO_POSE ==========
      auto waypoint_element = *waypoints_array.begin();
      auto waypoint_doc = waypoint_element.get_document().value;
      if (!waypoint_doc["type"]) {
        handle_error("Waypoint missing 'type' field");
        return;
      }
      auto waypoints_array = view["waypoints"].get_array().value;
      size_t num_waypoints = std::distance(waypoints_array.begin(), waypoints_array.end());
      std::string type = waypoint_doc["type"].get_string().value.to_string();
      
      if (type == "joint_values_absolute") {
        // Joint valuesで1個
        goal_msg.command = TmsRpExcavator::Goal::CMD_PLAN_TO_JOINTS;
        RCLCPP_INFO(this->get_logger(), "===================================================");
        RCLCPP_INFO(this->get_logger(), "  Waypoints: 1 (Single waypoint)");
        RCLCPP_INFO(this->get_logger(), "  Type: joint_values");
        RCLCPP_INFO(this->get_logger(), "  Command: CMD_PLAN_TO_JOINTS");
        RCLCPP_INFO(this->get_logger(), "==================================================="); 
        if (!waypoint_doc["data"]) {
          handle_error("Waypoint missing 'data' field");
          return;
        }
        auto data_doc = waypoint_doc["data"].get_document().value;
        tms_msg_rp::msg::TmsRpExcavatorJointValues target_joint_values;
        
        // 初期値を決定：previous_poseがあればそこから、なければcurrent_joint_states_から
        if (!goal_msg.previous_pose.empty()) {
          const auto& last_traj = goal_msg.previous_pose.back().joint_trajectory;
          if (!last_traj.joint_names.empty() && !last_traj.points.empty()) {
            target_joint_values.joint_names  = last_traj.joint_names;
            target_joint_values.joint_values = last_traj.points.back().positions;
            RCLCPP_INFO(this->get_logger(), "  Using previous plan's final pose as base");
          } else {
            target_joint_values.joint_names  = current_joint_states_.name;
            target_joint_values.joint_values = current_joint_states_.position;
            RCLCPP_INFO(this->get_logger(), "  Using current joint states as base");
          }
        } else {
          target_joint_values.joint_names  = current_joint_states_.name;
          target_joint_values.joint_values = current_joint_states_.position;
          RCLCPP_INFO(this->get_logger(), "  Using current joint states as base");
        }
        
        for (auto&& field : data_doc) {
          std::string joint_name = field.key().to_string();
          double joint_value = get_numeric_value(field);
          for (size_t i = 0; i < target_joint_values.joint_names.size(); ++i) {
            if (target_joint_values.joint_names[i] == joint_name) {
                target_joint_values.joint_values[i] = joint_value;
                break;
            }
          }
        }
        if (!binary_search_extreme_joint_value(goal_msg, target_joint_values, *param_response)) {
          handle_error("Failed to find valid joint values within limits");
          return;
        }
        level_bucket_if_trigger(target_joint_values);
        goal_msg.joint_values = target_joint_values;
        RCLCPP_INFO(this->get_logger(), "  Target: %zu joints specified", target_joint_values.joint_names.size());
        RCLCPP_INFO(this->get_logger(), "  Joint values:");
        for (size_t i = 0; i < target_joint_values.joint_names.size(); ++i) {
          RCLCPP_INFO(this->get_logger(), "    %s: %.3f rad (%.1f deg)",
                      target_joint_values.joint_names[i].c_str(),
                      target_joint_values.joint_values[i],
                      target_joint_values.joint_values[i] * 180.0 / M_PI);
        }

      } else if (type == "joint_values_relative") {
        // Joint valuesで1個（相対）
        goal_msg.command = TmsRpExcavator::Goal::CMD_PLAN_TO_JOINTS;
        RCLCPP_INFO(this->get_logger(), "===================================================");
        RCLCPP_INFO(this->get_logger(), "  Waypoints: 1 (Single waypoint)");
        RCLCPP_INFO(this->get_logger(), "  Type: joint_values (relative)");
        RCLCPP_INFO(this->get_logger(), "  Command: CMD_PLAN_TO_JOINTS");
        RCLCPP_INFO(this->get_logger(), "===================================================");
        if (!waypoint_doc["data"]) {
          handle_error("Waypoint missing 'data' field");
          return;
        }
        auto data_doc = waypoint_doc["data"].get_document().value;
        tms_msg_rp::msg::TmsRpExcavatorJointValues target_joint_values;

        // 初期値を決定：previous_poseがあればそこから、なければcurrent_joint_states_から
        if (!goal_msg.previous_pose.empty()) {
          const auto& last_traj = goal_msg.previous_pose.back().joint_trajectory;
          if (!last_traj.joint_names.empty() && !last_traj.points.empty()) {
            target_joint_values.joint_names  = last_traj.joint_names;
            target_joint_values.joint_values = last_traj.points.back().positions;
            RCLCPP_INFO(this->get_logger(), "  Using previous plan's final pose as base");
          } else {
            target_joint_values.joint_names  = current_joint_states_.name;
            target_joint_values.joint_values = current_joint_states_.position;
            RCLCPP_INFO(this->get_logger(), "  Using current joint states as base");
          }
        } else {
          target_joint_values.joint_names  = current_joint_states_.name;
          target_joint_values.joint_values = current_joint_states_.position;
          RCLCPP_INFO(this->get_logger(), "  Using current joint states as base");
        }
        
        for (auto&& field : data_doc) {
          std::string joint_name = field.key().to_string();
          double joint_value = get_numeric_value(field);
          for (size_t i = 0; i < target_joint_values.joint_names.size(); ++i) {
            if (target_joint_values.joint_names[i] == joint_name) {
                target_joint_values.joint_values[i] += joint_value;
                break;
            }
          }
        }
        level_bucket_if_trigger(target_joint_values);
        goal_msg.joint_values = target_joint_values;
        RCLCPP_INFO(this->get_logger(), "  Target: %zu joints specified (relative)", target_joint_values.joint_names.size());
        RCLCPP_INFO(this->get_logger(), "  Joint values:");
        for (size_t i = 0; i < target_joint_values.joint_names.size(); ++i) {
          RCLCPP_INFO(this->get_logger(), "    %s: %.3f rad (%.1f deg)",
                      target_joint_values.joint_names[i].c_str(),
                      target_joint_values.joint_values[i],
                      target_joint_values.joint_values[i] * 180.0 / M_PI);
        }
        
      } else if (type == "pose") {
        // Poseで1個
        goal_msg.command = TmsRpExcavator::Goal::CMD_PLAN_TO_POSE;
        RCLCPP_INFO(this->get_logger(), "===================================================");
        RCLCPP_INFO(this->get_logger(), "  Waypoints: 1 (Single waypoint)");
        RCLCPP_INFO(this->get_logger(), "  Type: pose");
        RCLCPP_INFO(this->get_logger(), "  Command: CMD_PLAN_TO_POSE");
        RCLCPP_INFO(this->get_logger(), "===================================================");
        
        if (!waypoint_doc["data"]) {
          handle_error("Waypoint missing 'data' field");
          return;
        }
        
        auto data_doc = waypoint_doc["data"].get_document().value;
        
        if (!data_doc["x"] || !data_doc["y"] || !data_doc["z"] || !data_doc["theta_w"]) {
          handle_error("Pose waypoint missing required fields (x, y, z, theta_w)");
          return;
        }
        
        double x = get_numeric_value(data_doc["x"]);
        double y = get_numeric_value(data_doc["y"]);
        double z = get_numeric_value(data_doc["z"]);
        double theta_w = get_numeric_value(data_doc["theta_w"]);
        
        Pose converted_pose;
        pose_converter.convertToXYZQuaternion(x, y, z, theta_w, converted_pose);
        
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = converted_pose.x;
        target_pose.position.y = converted_pose.y;
        target_pose.position.z = converted_pose.z;
        target_pose.orientation.x = converted_pose.qx;
        target_pose.orientation.y = converted_pose.qy;
        target_pose.orientation.z = converted_pose.qz;
        target_pose.orientation.w = converted_pose.qw;
        
        goal_msg.pose = target_pose;
        RCLCPP_INFO(this->get_logger(), "  Target pose: (%.2f, %.2f, %.2f)", x, y, z);
            
        } else {
            handle_error("Unknown waypoint type: " + type);
            return;
        }
      
    } else {
      // 2個の場合はない。今後消す。
      handle_error("Multiple waypoints not supported in this version");
      return;
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse waypoints: %s", e.what());
    handle_error("Failed to parse waypoints from DB");
    return;
  }
  
  if (!parse_constraints(goal_msg))
  {
    handle_error("Failed to parse constraints");
    return;
  }

  // Send goal
  auto send_goal_options = rclcpp_action::Client<TmsRpExcavator>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](const auto& goal_handle) { 
    goal_response_callback(goal_handle); 
  };
  send_goal_options.feedback_callback = [this](const auto tmp, const auto feedback) {
    feedback_callback(tmp, feedback);
  };
  send_goal_options.result_callback = [this, goal_handle](const auto& result) { 
    result_callback(goal_handle, result); 
  };

  RCLCPP_INFO(this->get_logger(), "Sending goal with command: %d", goal_msg.command);
  client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void PrimitiveExcavatorChangePosePlan::goal_response_callback(
    const GoalHandleTmsRpExcavator::SharedPtr& goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void PrimitiveExcavatorChangePosePlan::feedback_callback(
    const GoalHandleTmsRpExcavator::SharedPtr,
    const std::shared_ptr<const TmsRpExcavator::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Feedback: %s (progress: %.2f)", 
              feedback->state.c_str(), feedback->progress);
}

void PrimitiveExcavatorChangePosePlan::result_callback(
    const std::shared_ptr<GoalHandle> goal_handle,
    const GoalHandleTmsRpExcavator::WrappedResult& result)
{
  if (!goal_handle->is_active()) {
    RCLCPP_WARN(this->get_logger(), "Attempted to succeed an already succeeded goal");
    return;
  }

  auto result_to_leaf = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();
  
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      save_plan_to_db(result.result);
      result_to_leaf->result = true;
      goal_handle->succeed(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Primitive execution is succeeded");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      result_to_leaf->result = false;
      goal_handle->abort(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Primitive execution is aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      result_to_leaf->result = false;
      goal_handle->canceled(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Primitive execution is canceled");
      break;
    default:
      result_to_leaf->result = false;
      goal_handle->abort(result_to_leaf);
      RCLCPP_INFO(this->get_logger(), "Unknown result code");
      break;
  }
}

void PrimitiveExcavatorChangePosePlan::save_plan_to_db(const TmsRpExcavator::Result::SharedPtr& result)
{
  try {
    // Check if plan has valid data
    if (result->plan.joint_trajectory.joint_names.empty() && 
        result->plan.multi_dof_joint_trajectory.joint_names.empty()) {
      RCLCPP_WARN(this->get_logger(), "Plan is empty, nothing to save");
      return;
    }
    
    bsoncxx::builder::basic::document plan_doc;
    
    // ========== joint_trajectory ==========
    if (!result->plan.joint_trajectory.joint_names.empty()) {
      bsoncxx::builder::basic::document joint_traj_doc;
      
      bsoncxx::builder::basic::array joint_names_array;
      for (const auto& name : result->plan.joint_trajectory.joint_names) {
        joint_names_array.append(name);
      }
      joint_traj_doc.append(bsoncxx::builder::basic::kvp("joint_names", joint_names_array));
      
      bsoncxx::builder::basic::array points_array;
      for (const auto& point : result->plan.joint_trajectory.points) {
        bsoncxx::builder::basic::document point_doc;
        
        bsoncxx::builder::basic::array positions_array;
        for (const auto& pos : point.positions) {
          positions_array.append(pos);
        }
        point_doc.append(bsoncxx::builder::basic::kvp("positions", positions_array));
        
        if (!point.velocities.empty()) {
          bsoncxx::builder::basic::array velocities_array;
          for (const auto& vel : point.velocities) {
            velocities_array.append(vel);
          }
          point_doc.append(bsoncxx::builder::basic::kvp("velocities", velocities_array));
        }
        
        if (!point.accelerations.empty()) {
          bsoncxx::builder::basic::array accelerations_array;
          for (const auto& acc : point.accelerations) {
            accelerations_array.append(acc);
          }
          point_doc.append(bsoncxx::builder::basic::kvp("accelerations", accelerations_array));
        }
        
        point_doc.append(bsoncxx::builder::basic::kvp("time_from_start", 
          bsoncxx::builder::basic::make_document(
            bsoncxx::builder::basic::kvp("sec", static_cast<int32_t>(point.time_from_start.sec)),
            bsoncxx::builder::basic::kvp("nanosec", static_cast<int32_t>(point.time_from_start.nanosec))
          )
        ));
        
        points_array.append(point_doc);
      }
      joint_traj_doc.append(bsoncxx::builder::basic::kvp("points", points_array));
      plan_doc.append(bsoncxx::builder::basic::kvp("joint_trajectory", joint_traj_doc));
    }
    
    // ========== multi_dof_joint_trajectory ==========
    if (!result->plan.multi_dof_joint_trajectory.joint_names.empty()) {
      bsoncxx::builder::basic::document multi_dof_doc;
      
      bsoncxx::builder::basic::array joint_names_array;
      for (const auto& name : result->plan.multi_dof_joint_trajectory.joint_names) {
        joint_names_array.append(name);
      }
      multi_dof_doc.append(bsoncxx::builder::basic::kvp("joint_names", joint_names_array));
      
      bsoncxx::builder::basic::array points_array;
      for (const auto& point : result->plan.multi_dof_joint_trajectory.points) {
        bsoncxx::builder::basic::document point_doc;
        
        // transforms
        if (!point.transforms.empty()) {
          bsoncxx::builder::basic::array transforms_array;
          for (const auto& transform : point.transforms) {
            transforms_array.append(bsoncxx::builder::basic::make_document(
              bsoncxx::builder::basic::kvp("translation", bsoncxx::builder::basic::make_document(
                bsoncxx::builder::basic::kvp("x", transform.translation.x),
                bsoncxx::builder::basic::kvp("y", transform.translation.y),
                bsoncxx::builder::basic::kvp("z", transform.translation.z)
              )),
              bsoncxx::builder::basic::kvp("rotation", bsoncxx::builder::basic::make_document(
                bsoncxx::builder::basic::kvp("x", transform.rotation.x),
                bsoncxx::builder::basic::kvp("y", transform.rotation.y),
                bsoncxx::builder::basic::kvp("z", transform.rotation.z),
                bsoncxx::builder::basic::kvp("w", transform.rotation.w)
              ))
            ));
          }
          point_doc.append(bsoncxx::builder::basic::kvp("transforms", transforms_array));
        }
        
        // velocities
        if (!point.velocities.empty()) {
          bsoncxx::builder::basic::array velocities_array;
          for (const auto& twist : point.velocities) {
            velocities_array.append(bsoncxx::builder::basic::make_document(
              bsoncxx::builder::basic::kvp("linear", bsoncxx::builder::basic::make_document(
                bsoncxx::builder::basic::kvp("x", twist.linear.x),
                bsoncxx::builder::basic::kvp("y", twist.linear.y),
                bsoncxx::builder::basic::kvp("z", twist.linear.z)
              )),
              bsoncxx::builder::basic::kvp("angular", bsoncxx::builder::basic::make_document(
                bsoncxx::builder::basic::kvp("x", twist.angular.x),
                bsoncxx::builder::basic::kvp("y", twist.angular.y),
                bsoncxx::builder::basic::kvp("z", twist.angular.z)
              ))
            ));
          }
          point_doc.append(bsoncxx::builder::basic::kvp("velocities", velocities_array));
        }
        
        // accelerations
        if (!point.accelerations.empty()) {
          bsoncxx::builder::basic::array accelerations_array;
          for (const auto& twist : point.accelerations) {
            accelerations_array.append(bsoncxx::builder::basic::make_document(
              bsoncxx::builder::basic::kvp("linear", bsoncxx::builder::basic::make_document(
                bsoncxx::builder::basic::kvp("x", twist.linear.x),
                bsoncxx::builder::basic::kvp("y", twist.linear.y),
                bsoncxx::builder::basic::kvp("z", twist.linear.z)
              )),
              bsoncxx::builder::basic::kvp("angular", bsoncxx::builder::basic::make_document(
                bsoncxx::builder::basic::kvp("x", twist.angular.x),
                bsoncxx::builder::basic::kvp("y", twist.angular.y),
                bsoncxx::builder::basic::kvp("z", twist.angular.z)
              ))
            ));
          }
          point_doc.append(bsoncxx::builder::basic::kvp("accelerations", accelerations_array));
        }
        
        point_doc.append(bsoncxx::builder::basic::kvp("time_from_start", 
          bsoncxx::builder::basic::make_document(
            bsoncxx::builder::basic::kvp("sec", static_cast<int32_t>(point.time_from_start.sec)),
            bsoncxx::builder::basic::kvp("nanosec", static_cast<int32_t>(point.time_from_start.nanosec))
          )
        ));
        
        points_array.append(point_doc);
      }
      multi_dof_doc.append(bsoncxx::builder::basic::kvp("points", points_array));
      plan_doc.append(bsoncxx::builder::basic::kvp("multi_dof_joint_trajectory", multi_dof_doc));
    }
    
    std::string plan_json = bsoncxx::to_json(plan_doc.view());
    
    if(UpdateParamInDBFromJson(used_model_name_, used_record_name_, "plan", plan_json))
    {
      RCLCPP_INFO(this->get_logger(), "Successfully saved plan to database");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to save plan to database");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while saving plan: %s", e.what());
  }
}

bool PrimitiveExcavatorChangePosePlan::parse_previous_plan(TmsRpExcavator::Goal& goal_msg)
{
  if (previous_target_record_name_.empty() || previous_param_from_db_.empty())
  {
    RCLCPP_INFO(this->get_logger(), "No previous plan specified, will plan from current pose");
    return true;
  }

  RCLCPP_INFO(this->get_logger(), "Loading previous plan from record: %s", previous_target_record_name_.c_str());

  try
  {
    if (!previous_param_from_db_.count("plan"))
    {
      RCLCPP_INFO(this->get_logger(), "Previous record has no plan, will plan from current pose");
      return true;
    }

    // DBから来る "plan" は {"plan": {...}} でラップされている
    auto plan_doc  = bsoncxx::from_json(previous_param_from_db_["plan"]);
    auto plan_view = plan_doc.view();

    // planキーで1段ラップされていたら剥がす
    if (plan_view["plan"] && plan_view["plan"].type() == bsoncxx::type::k_document)
    {
      plan_view = plan_view["plan"].get_document().value;
    }

    // 単一のRobotTrajectoryを作成
    moveit_msgs::msg::RobotTrajectory robot_trajectory;

    // ---------------- joint_trajectory ----------------
    if (plan_view["joint_trajectory"] && plan_view["joint_trajectory"].type() == bsoncxx::type::k_document)
    {
      auto joint_traj_doc = plan_view["joint_trajectory"].get_document().value;

      if (joint_traj_doc["joint_names"] && joint_traj_doc["joint_names"].type() == bsoncxx::type::k_array)
      {
        auto joint_names_array = joint_traj_doc["joint_names"].get_array().value;
        for (auto&& name : joint_names_array)
        {
          if (name.type() == bsoncxx::type::k_utf8)
            robot_trajectory.joint_trajectory.joint_names.push_back(name.get_string().value.to_string());
        }
      }

      if (joint_traj_doc["points"] && joint_traj_doc["points"].type() == bsoncxx::type::k_array)
      {
        auto points_array = joint_traj_doc["points"].get_array().value;
        for (auto&& point_element : points_array)
        {
          if (point_element.type() != bsoncxx::type::k_document) continue;
          auto point_doc = point_element.get_document().value;

          trajectory_msgs::msg::JointTrajectoryPoint point;

          if (point_doc["positions"] && point_doc["positions"].type() == bsoncxx::type::k_array)
          {
            auto arr = point_doc["positions"].get_array().value;
            for (auto&& v : arr) point.positions.push_back(get_numeric_value(v));
          }

          if (point_doc["velocities"] && point_doc["velocities"].type() == bsoncxx::type::k_array)
          {
            auto arr = point_doc["velocities"].get_array().value;
            for (auto&& v : arr) point.velocities.push_back(get_numeric_value(v));
          }

          if (point_doc["accelerations"] && point_doc["accelerations"].type() == bsoncxx::type::k_array)
          {
            auto arr = point_doc["accelerations"].get_array().value;
            for (auto&& v : arr) point.accelerations.push_back(get_numeric_value(v));
          }

          if (point_doc["time_from_start"] && point_doc["time_from_start"].type() == bsoncxx::type::k_document)
          {
            auto time_doc = point_doc["time_from_start"].get_document().value;
            if (time_doc["sec"]     && time_doc["sec"].type() == bsoncxx::type::k_int32)
              point.time_from_start.sec     = time_doc["sec"].get_int32().value;
            if (time_doc["nanosec"] && time_doc["nanosec"].type() == bsoncxx::type::k_int32)
              point.time_from_start.nanosec = time_doc["nanosec"].get_int32().value;
          }

          robot_trajectory.joint_trajectory.points.push_back(point);
        }
      }
    }

    // ---------------- multi_dof_joint_trajectory ----------------
    if (plan_view["multi_dof_joint_trajectory"] &&
        plan_view["multi_dof_joint_trajectory"].type() == bsoncxx::type::k_document)
    {
      auto multi_dof_doc = plan_view["multi_dof_joint_trajectory"].get_document().value;

      if (multi_dof_doc["joint_names"] && multi_dof_doc["joint_names"].type() == bsoncxx::type::k_array)
      {
        auto arr = multi_dof_doc["joint_names"].get_array().value;
        for (auto&& name : arr)
        {
          if (name.type() == bsoncxx::type::k_utf8)
            robot_trajectory.multi_dof_joint_trajectory.joint_names.push_back(name.get_string().value.to_string());
        }
      }

      if (multi_dof_doc["points"] && multi_dof_doc["points"].type() == bsoncxx::type::k_array)
      {
        auto points_array = multi_dof_doc["points"].get_array().value;
        for (auto&& point_element : points_array)
        {
          if (point_element.type() != bsoncxx::type::k_document) continue;
          auto point_doc = point_element.get_document().value;

          trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;

          if (point_doc["transforms"] && point_doc["transforms"].type() == bsoncxx::type::k_array)
          {
            auto arr = point_doc["transforms"].get_array().value;
            for (auto&& t : arr)
            {
              if (t.type() != bsoncxx::type::k_document) continue;
              auto td = t.get_document().value;

              geometry_msgs::msg::Transform transform;

              if (td["translation"] && td["translation"].type() == bsoncxx::type::k_document)
              {
                auto tr = td["translation"].get_document().value;
                if (tr["x"]) transform.translation.x = get_numeric_value(tr["x"]);
                if (tr["y"]) transform.translation.y = get_numeric_value(tr["y"]);
                if (tr["z"]) transform.translation.z = get_numeric_value(tr["z"]);
              }

              if (td["rotation"] && td["rotation"].type() == bsoncxx::type::k_document)
              {
                auto ro = td["rotation"].get_document().value;
                if (ro["x"]) transform.rotation.x = get_numeric_value(ro["x"]);
                if (ro["y"]) transform.rotation.y = get_numeric_value(ro["y"]);
                if (ro["z"]) transform.rotation.z = get_numeric_value(ro["z"]);
                if (ro["w"]) transform.rotation.w = get_numeric_value(ro["w"]);
              }

              point.transforms.push_back(transform);
            }
          }

          if (point_doc["velocities"] && point_doc["velocities"].type() == bsoncxx::type::k_array)
          {
            auto arr = point_doc["velocities"].get_array().value;
            for (auto&& v : arr)
            {
              if (v.type() != bsoncxx::type::k_document) continue;
              auto vd = v.get_document().value;

              geometry_msgs::msg::Twist twist;

              if (vd["linear"] && vd["linear"].type() == bsoncxx::type::k_document)
              {
                auto li = vd["linear"].get_document().value;
                if (li["x"]) twist.linear.x = get_numeric_value(li["x"]);
                if (li["y"]) twist.linear.y = get_numeric_value(li["y"]);
                if (li["z"]) twist.linear.z = get_numeric_value(li["z"]);
              }
              if (vd["angular"] && vd["angular"].type() == bsoncxx::type::k_document)
              {
                auto an = vd["angular"].get_document().value;
                if (an["x"]) twist.angular.x = get_numeric_value(an["x"]);
                if (an["y"]) twist.angular.y = get_numeric_value(an["y"]);
                if (an["z"]) twist.angular.z = get_numeric_value(an["z"]);
              }

              point.velocities.push_back(twist);
            }
          }

          if (point_doc["accelerations"] && point_doc["accelerations"].type() == bsoncxx::type::k_array)
          {
            auto arr = point_doc["accelerations"].get_array().value;
            for (auto&& a : arr)
            {
              if (a.type() != bsoncxx::type::k_document) continue;
              auto ad = a.get_document().value;

              geometry_msgs::msg::Twist twist;

              if (ad["linear"] && ad["linear"].type() == bsoncxx::type::k_document)
              {
                auto li = ad["linear"].get_document().value;
                if (li["x"]) twist.linear.x = get_numeric_value(li["x"]);
                if (li["y"]) twist.linear.y = get_numeric_value(li["y"]);
                if (li["z"]) twist.linear.z = get_numeric_value(li["z"]);
              }
              if (ad["angular"] && ad["angular"].type() == bsoncxx::type::k_document)
              {
                auto an = ad["angular"].get_document().value;
                if (an["x"]) twist.angular.x = get_numeric_value(an["x"]);
                if (an["y"]) twist.angular.y = get_numeric_value(an["y"]);
                if (an["z"]) twist.angular.z = get_numeric_value(an["z"]);
              }

              point.accelerations.push_back(twist);
            }
          }

          if (point_doc["time_from_start"] && point_doc["time_from_start"].type() == bsoncxx::type::k_document)
          {
            auto time_doc = point_doc["time_from_start"].get_document().value;
            if (time_doc["sec"]     && time_doc["sec"].type() == bsoncxx::type::k_int32)
              point.time_from_start.sec     = time_doc["sec"].get_int32().value;
            if (time_doc["nanosec"] && time_doc["nanosec"].type() == bsoncxx::type::k_int32)
              point.time_from_start.nanosec = time_doc["nanosec"].get_int32().value;
          }

          robot_trajectory.multi_dof_joint_trajectory.points.push_back(point);
        }
      }
    }

    // 読み込んだtrajectoryを追加
    goal_msg.previous_pose.push_back(robot_trajectory);

    RCLCPP_INFO(this->get_logger(),
                "Successfully loaded previous plan (joint_trajectory: %zu points, multi_dof: %zu points)",
                robot_trajectory.joint_trajectory.points.size(),
                robot_trajectory.multi_dof_joint_trajectory.points.size());
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse previous plan: %s", e.what());
    return false;
  }
}

bool PrimitiveExcavatorChangePosePlan::parse_constraints(TmsRpExcavator::Goal& goal_msg)
{
  try
  {
    // constraints が無ければ何もしない（成功扱い）
    if (!param_from_db_.count("constraints")) return true;

    auto doc  = bsoncxx::from_json(param_from_db_["constraints"]);
    auto view = doc.view();
    if (!view["constraints"]) return true;

    RCLCPP_INFO(this->get_logger(), "Parsing constraints from JSON");
    auto constraints_doc = view["constraints"].get_document().value;

    // -------------------------
    // joint_constraints
    // -------------------------
    if (constraints_doc["joint_constraints"])
    {
      auto joint_constraints = constraints_doc["joint_constraints"].get_array().value;
      for (auto&& jc : joint_constraints)
      {
        auto jc_doc = jc.get_document().value;
        moveit_msgs::msg::JointConstraint joint_constraint;

        if (jc_doc["joint_name"])
          joint_constraint.joint_name = jc_doc["joint_name"].get_string().value.to_string();
        if (jc_doc["position"]) {
          // positionが999.0の場合はprevious_poseまたはcurrent_joint_states_を基準に制約を設定する
          double pos_val = get_numeric_value(jc_doc["position"]);
          if (std::abs(pos_val - 999.0) < 1e-6) {

            if (!goal_msg.previous_pose.empty()) {
              // Try previous_pose
              const auto& traj = goal_msg.previous_pose.back().joint_trajectory;
              if (!traj.joint_names.empty() && !traj.points.empty()) {
                const auto& last_pt = traj.points.back();
                auto it = std::find(traj.joint_names.begin(), traj.joint_names.end(), joint_constraint.joint_name);
                if (it != traj.joint_names.end()) {
                  size_t index = std::distance(traj.joint_names.begin(), it);
                  if (index < last_pt.positions.size()) {
                    joint_constraint.position = last_pt.positions[index];
                    RCLCPP_INFO(this->get_logger(), "Set joint '%s' position to previous_pose value: %f",
                                joint_constraint.joint_name.c_str(), joint_constraint.position);
                  } else {
                    RCLCPP_WARN(this->get_logger(), "Joint '%s' found in previous_pose but positions size mismatch",
                                joint_constraint.joint_name.c_str());
                    return true;
                  }
                } else {
                  RCLCPP_WARN(this->get_logger(), "Joint '%s' not found in previous_pose",
                              joint_constraint.joint_name.c_str());
                  return true;
                }
              } else {
                RCLCPP_WARN(this->get_logger(), "previous_pose exists but is empty");
                return true;
              }
            } else {
              // Try current_joint_states
              auto it = std::find(current_joint_states_.name.begin(), current_joint_states_.name.end(), joint_constraint.joint_name);
              if (it != current_joint_states_.name.end()) {
                size_t index = std::distance(current_joint_states_.name.begin(), it);
                if (index < current_joint_states_.position.size()) {
                  joint_constraint.position = current_joint_states_.position[index];
                  RCLCPP_INFO(this->get_logger(), "Set joint '%s' position to current value: %f",
                              joint_constraint.joint_name.c_str(), joint_constraint.position);
                } else {
                  RCLCPP_WARN(this->get_logger(), "Joint '%s' found in current_joint_states_ but positions size mismatch",
                              joint_constraint.joint_name.c_str());
                }
              } else {
                RCLCPP_WARN(this->get_logger(), "Joint '%s' not found in current_joint_states_",
                            joint_constraint.joint_name.c_str());
              }
            }
          } else {
            RCLCPP_INFO(this->get_logger(), "Retrieved joint position from DB: %f", pos_val);
            joint_constraint.position = pos_val;
          }
        }
        if (jc_doc["tolerance_above"])
          joint_constraint.tolerance_above = get_numeric_value(jc_doc["tolerance_above"]);
        if (jc_doc["tolerance_below"])
          joint_constraint.tolerance_below = get_numeric_value(jc_doc["tolerance_below"]);
        if (jc_doc["weight"])
          joint_constraint.weight = get_numeric_value(jc_doc["weight"]);

        goal_msg.constraints.joint_constraints.push_back(joint_constraint);
      }
      RCLCPP_INFO(this->get_logger(), "Added %zu joint constraints",
                  goal_msg.constraints.joint_constraints.size());
    }

    // -------------------------
    // position_constraints
    // -------------------------
    if (constraints_doc["position_constraints"])
    {
      auto position_constraints = constraints_doc["position_constraints"].get_array().value;
      for (auto&& pc : position_constraints)
      {
        auto pc_doc = pc.get_document().value;
        moveit_msgs::msg::PositionConstraint position_constraint;

        // header.frame_id
        if (pc_doc["header"] && pc_doc["header"].get_document().value["frame_id"])
        {
          position_constraint.header.frame_id =
              pc_doc["header"].get_document().value["frame_id"].get_string().value.to_string();
        }

        // link_name
        if (pc_doc["link_name"])
          position_constraint.link_name = pc_doc["link_name"].get_string().value.to_string();

        // target_point_offset
        if (pc_doc["target_point_offset"])
        {
          auto offset = pc_doc["target_point_offset"].get_document().value;
          if (offset["x"]) position_constraint.target_point_offset.x = get_numeric_value(offset["x"]);
          if (offset["y"]) position_constraint.target_point_offset.y = get_numeric_value(offset["y"]);
          if (offset["z"]) position_constraint.target_point_offset.z = get_numeric_value(offset["z"]);
        }

        // constraint_region
        if (pc_doc["constraint_region"])
        {
          auto region = pc_doc["constraint_region"].get_document().value;

          // primitives
          if (region["primitives"])
          {
            auto primitives = region["primitives"].get_array().value;
            for (auto&& prim : primitives)
            {
              auto prim_doc = prim.get_document().value;
              shape_msgs::msg::SolidPrimitive solid_primitive;

              if (prim_doc["type"])
              {
                if (prim_doc["type"].type() == bsoncxx::type::k_int32)
                {
                  solid_primitive.type = prim_doc["type"].get_int32().value;
                }
                else if (prim_doc["type"].type() == bsoncxx::type::k_int64)
                {
                  solid_primitive.type = static_cast<uint8_t>(prim_doc["type"].get_int64().value);
                }
              }

              if (prim_doc["dimensions"])
              {
                auto dimensions = prim_doc["dimensions"].get_array().value;
                for (auto&& dim : dimensions)
                  solid_primitive.dimensions.push_back(get_numeric_value(dim));
              }

              position_constraint.constraint_region.primitives.push_back(solid_primitive);
            }
          }

          // primitive_poses
          if (region["primitive_poses"])
          {
            auto poses = region["primitive_poses"].get_array().value;
            for (auto&& pose : poses)
            {
              auto pose_doc = pose.get_document().value;
              geometry_msgs::msg::Pose geo_pose;

              if (pose_doc["position"])
              {
                auto pos = pose_doc["position"].get_document().value;
                if (pos["x"]) geo_pose.position.x = get_numeric_value(pos["x"]);
                if (pos["y"]) geo_pose.position.y = get_numeric_value(pos["y"]);
                if (pos["z"]) geo_pose.position.z = get_numeric_value(pos["z"]);
              }

              if (pose_doc["orientation"])
              {
                auto ori = pose_doc["orientation"].get_document().value;
                if (ori["x"]) geo_pose.orientation.x = get_numeric_value(ori["x"]);
                if (ori["y"]) geo_pose.orientation.y = get_numeric_value(ori["y"]);
                if (ori["z"]) geo_pose.orientation.z = get_numeric_value(ori["z"]);
                if (ori["w"]) geo_pose.orientation.w = get_numeric_value(ori["w"]);
              }

              position_constraint.constraint_region.primitive_poses.push_back(geo_pose);
            }
          }
        }

        // weight
        if (pc_doc["weight"])
        {
          if (pc_doc["weight"].type() == bsoncxx::type::k_double)
            position_constraint.weight = get_numeric_value(pc_doc["weight"]);
          else if (pc_doc["weight"].type() == bsoncxx::type::k_int32 || pc_doc["weight"].type() == bsoncxx::type::k_int64)
            position_constraint.weight = get_numeric_value(pc_doc["weight"]);
        }

        goal_msg.constraints.position_constraints.push_back(position_constraint);
      }

      RCLCPP_INFO(this->get_logger(), "Added %zu position constraints",
                  goal_msg.constraints.position_constraints.size());
    }

    // -------------------------
    // orientation_constraints
    // -------------------------
    if (constraints_doc["orientation_constraints"])
    {
      auto orientation_constraints = constraints_doc["orientation_constraints"].get_array().value;
      for (auto&& oc : orientation_constraints)
      {
        auto oc_doc = oc.get_document().value;
        moveit_msgs::msg::OrientationConstraint orientation_constraint;

        // header.frame_id
        if (oc_doc["header"] && oc_doc["header"].get_document().value["frame_id"])
        {
          orientation_constraint.header.frame_id =
              oc_doc["header"].get_document().value["frame_id"].get_string().value.to_string();
        }

        // link_name
        if (oc_doc["link_name"])
          orientation_constraint.link_name = oc_doc["link_name"].get_string().value.to_string();

        // orientation
        if (oc_doc["orientation"])
        {
          auto ori = oc_doc["orientation"].get_document().value;
          if (ori["x"]) orientation_constraint.orientation.x = get_numeric_value(ori["x"]);
          if (ori["y"]) orientation_constraint.orientation.y = get_numeric_value(ori["y"]);
          if (ori["z"]) orientation_constraint.orientation.z = get_numeric_value(ori["z"]);
          if (ori["w"]) orientation_constraint.orientation.w = get_numeric_value(ori["w"]);
        }

        // tolerances
        if (oc_doc["absolute_x_axis_tolerance"])
          orientation_constraint.absolute_x_axis_tolerance = get_numeric_value(oc_doc["absolute_x_axis_tolerance"]);
        if (oc_doc["absolute_y_axis_tolerance"])
          orientation_constraint.absolute_y_axis_tolerance = get_numeric_value(oc_doc["absolute_y_axis_tolerance"]);
        if (oc_doc["absolute_z_axis_tolerance"])
          orientation_constraint.absolute_z_axis_tolerance = get_numeric_value(oc_doc["absolute_z_axis_tolerance"]);

        // weight
        if (oc_doc["weight"])
        {
          if (oc_doc["weight"].type() == bsoncxx::type::k_double)
            orientation_constraint.weight = get_numeric_value(oc_doc["weight"]);
          else if (oc_doc["weight"].type() == bsoncxx::type::k_int32 || oc_doc["weight"].type() == bsoncxx::type::k_int64)
            orientation_constraint.weight = get_numeric_value(oc_doc["weight"]);
        }

        goal_msg.constraints.orientation_constraints.push_back(orientation_constraint);
      }

      RCLCPP_INFO(this->get_logger(), "Added %zu orientation constraints",
                  goal_msg.constraints.orientation_constraints.size());
    }

    // -------------------------
    // visibility_constraints
    // -------------------------
    if (constraints_doc["visibility_constraints"])
    {
      auto visibility_constraints = constraints_doc["visibility_constraints"].get_array().value;
      for (auto&& vc : visibility_constraints)
      {
        auto vc_doc = vc.get_document().value;
        moveit_msgs::msg::VisibilityConstraint visibility_constraint;

        // target_radius
        if (vc_doc["target_radius"])
          visibility_constraint.target_radius = get_numeric_value(vc_doc["target_radius"]);

        // target_pose
        if (vc_doc["target_pose"])
        {
          auto target_pose = vc_doc["target_pose"].get_document().value;

          if (target_pose["header"] && target_pose["header"].get_document().value["frame_id"])
          {
            visibility_constraint.target_pose.header.frame_id =
                target_pose["header"].get_document().value["frame_id"].get_string().value.to_string();
          }

          if (target_pose["pose"])
          {
            auto pose = target_pose["pose"].get_document().value;

            if (pose["position"])
            {
              auto pos = pose["position"].get_document().value;
              if (pos["x"]) visibility_constraint.target_pose.pose.position.x = get_numeric_value(pos["x"]);
              if (pos["y"]) visibility_constraint.target_pose.pose.position.y = get_numeric_value(pos["y"]);
              if (pos["z"]) visibility_constraint.target_pose.pose.position.z = get_numeric_value(pos["z"]);
            }

            if (pose["orientation"])
            {
              auto ori = pose["orientation"].get_document().value;
              if (ori["x"]) visibility_constraint.target_pose.pose.orientation.x = get_numeric_value(ori["x"]);
              if (ori["y"]) visibility_constraint.target_pose.pose.orientation.y = get_numeric_value(ori["y"]);
              if (ori["z"]) visibility_constraint.target_pose.pose.orientation.z = get_numeric_value(ori["z"]);
              if (ori["w"]) visibility_constraint.target_pose.pose.orientation.w = get_numeric_value(ori["w"]);
            }
          }
        }

        // cone_sides
        if (vc_doc["cone_sides"])
        {
          if (vc_doc["cone_sides"].type() == bsoncxx::type::k_int32)
            visibility_constraint.cone_sides = vc_doc["cone_sides"].get_int32().value;
          else if (vc_doc["cone_sides"].type() == bsoncxx::type::k_int64)
            visibility_constraint.cone_sides = static_cast<int32_t>(vc_doc["cone_sides"].get_int64().value);
        }

        // sensor_pose
        if (vc_doc["sensor_pose"])
        {
          auto sensor_pose = vc_doc["sensor_pose"].get_document().value;

          if (sensor_pose["header"] && sensor_pose["header"].get_document().value["frame_id"])
          {
            visibility_constraint.sensor_pose.header.frame_id =
                sensor_pose["header"].get_document().value["frame_id"].get_string().value.to_string();
          }

          if (sensor_pose["pose"])
          {
            auto pose = sensor_pose["pose"].get_document().value;

            if (pose["position"])
            {
              auto pos = pose["position"].get_document().value;
              if (pos["x"]) visibility_constraint.sensor_pose.pose.position.x = get_numeric_value(pos["x"]);
              if (pos["y"]) visibility_constraint.sensor_pose.pose.position.y = get_numeric_value(pos["y"]);
              if (pos["z"]) visibility_constraint.sensor_pose.pose.position.z = get_numeric_value(pos["z"]);
            }

            if (pose["orientation"])
            {
              auto ori = pose["orientation"].get_document().value;
              if (ori["x"]) visibility_constraint.sensor_pose.pose.orientation.x = get_numeric_value(ori["x"]);
              if (ori["y"]) visibility_constraint.sensor_pose.pose.orientation.y = get_numeric_value(ori["y"]);
              if (ori["z"]) visibility_constraint.sensor_pose.pose.orientation.z = get_numeric_value(ori["z"]);
              if (ori["w"]) visibility_constraint.sensor_pose.pose.orientation.w = get_numeric_value(ori["w"]);
            }
          }
        }

        // angles
        if (vc_doc["max_view_angle"])
          visibility_constraint.max_view_angle = get_numeric_value(vc_doc["max_view_angle"]);
        if (vc_doc["max_range_angle"])
          visibility_constraint.max_range_angle = get_numeric_value(vc_doc["max_range_angle"]);

        // sensor_view_direction
        if (vc_doc["sensor_view_direction"])
        {
          if (vc_doc["sensor_view_direction"].type() == bsoncxx::type::k_int32)
            visibility_constraint.sensor_view_direction = vc_doc["sensor_view_direction"].get_int32().value;
          else if (vc_doc["sensor_view_direction"].type() == bsoncxx::type::k_int64)
            visibility_constraint.sensor_view_direction = static_cast<uint8_t>(vc_doc["sensor_view_direction"].get_int64().value);
        }

        // weight
        if (vc_doc["weight"])
        {
          if (vc_doc["weight"].type() == bsoncxx::type::k_double)
            visibility_constraint.weight = get_numeric_value(vc_doc["weight"]);
          else if (vc_doc["weight"].type() == bsoncxx::type::k_int32 || vc_doc["weight"].type() == bsoncxx::type::k_int64)
            visibility_constraint.weight = get_numeric_value(vc_doc["weight"]);
        }

        goal_msg.constraints.visibility_constraints.push_back(visibility_constraint);
      }

      RCLCPP_INFO(this->get_logger(), "Added %zu visibility constraints",
                  goal_msg.constraints.visibility_constraints.size());
    }

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse constraints: %s", e.what());
    return false;
  }
}

bool PrimitiveExcavatorChangePosePlan::binary_search_extreme_joint_value(
  const TmsRpExcavator::Goal& goal_msg,
  tms_msg_rp::msg::TmsRpExcavatorJointValues& target_joint_values,
  const tms_msg_rp::srv::TmsRpExcavatorParamGet::Response& res)
{
  int numuntillimit = 0;
  std::string serch_joint_name_ = "";
  int direction = 0;  // +1: upper limit, -1: lower limit
  double original = 0.0;
  size_t joint_idx = 0;
  tms_msg_rp::msg::TmsRpExcavatorJointValues best_joint_values;
  bool found_any_success = false;

  for (size_t i = 0; i < target_joint_values.joint_values.size(); ++i) {
      if (target_joint_values.joint_values[i] >= 999 || target_joint_values.joint_values[i] <= -999) {
          numuntillimit += 1;
          serch_joint_name_ = target_joint_values.joint_names[i];
          direction = (target_joint_values.joint_values[i] >= 999) ? 1 : -1;
          joint_idx = i;
      }
  }
  if (numuntillimit == 0) {
    RCLCPP_INFO(this->get_logger(), "No joint with 'until limit' (999) found, skipping binary search");
    return true;
  } else if (numuntillimit > 1) {
    RCLCPP_WARN(this->get_logger(), "Multiple joints with 'until limit' (999) detected, but handling multiple is not supported yet. Found: %d", numuntillimit);
    return false;
  }

  tms_msg_rp::msg::TmsRpExcavatorJointValues previous_joint_values;
  if (!goal_msg.previous_pose.empty()) {
    const auto& traj = goal_msg.previous_pose.back().joint_trajectory;
    if (traj.joint_names.empty() || traj.points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "previous_pose last trajectory has no joint_names or points");
      return false;
    }
    const auto& last_pt = traj.points.back();
    if (last_pt.positions.size() != traj.joint_names.size()) {
      RCLCPP_ERROR(this->get_logger(),
        "Size mismatch: joint_names=%zu positions=%zu",
        traj.joint_names.size(), last_pt.positions.size());
      return false;
    }
    previous_joint_values.joint_names  = traj.joint_names;
    previous_joint_values.joint_values = last_pt.positions;
  }

  const auto& joint_names = res.joint_names;
  double lowest, highest;
  size_t limit_joint_idx = std::find(joint_names.begin(), joint_names.end(), serch_joint_name_) - joint_names.begin();
  if (limit_joint_idx >= joint_names.size()) {
    RCLCPP_ERROR(this->get_logger(), "Joint '%s' not found in response", serch_joint_name_.c_str());
    return false;
  }
  if (previous_joint_values.joint_names.empty()) {
    original = current_joint_states_.position[joint_idx];
  } else {
    original = previous_joint_values.joint_values[joint_idx];
  }
  if (direction > 0)  { lowest = original; highest = res.max_positions[limit_joint_idx]; }
  else                { lowest = res.min_positions[limit_joint_idx]; highest = original; }

  int iteration = 0;
  while (std::abs(highest - lowest) > search_precision_) {
    const double mid = (lowest + highest) / 2.0;
    iteration++;

    auto test_joint_values = target_joint_values;
    test_joint_values.joint_values[joint_idx] = mid;

    level_bucket_if_trigger(test_joint_values);

    auto excavator_goal = TmsRpExcavator::Goal();
    excavator_goal.command = TmsRpExcavator::Goal::CMD_PLAN_TO_JOINTS;
    excavator_goal.planning_group = planning_group_;
    excavator_goal.joint_values = test_joint_values;
    excavator_goal.constraints = goal_msg.constraints;
    excavator_goal.previous_pose = goal_msg.previous_pose;

    TmsRpExcavator::Result::SharedPtr result;
    const bool success = call_excavator_action_sync(excavator_goal, result);

    if (success) {
      best_joint_values = test_joint_values;
      found_any_success = true;
      if (direction > 0) {
        lowest = mid;
      } else {
        highest = mid;          // min側へ（より小さい値へ）
      }
    } else {
      // failなら「戻す」方向に狭める
      if (direction > 0) {
        highest = mid;
      } else {
        lowest = mid;
      }
    }
  }

  if (!found_any_success) {
    RCLCPP_WARN(this->get_logger(), "Binary search failed to find any valid joint value for joint '%s'", serch_joint_name_.c_str());
    return false;
  }
  target_joint_values.joint_values[joint_idx] = best_joint_values.joint_values[joint_idx];
  RCLCPP_INFO(this->get_logger(),
              "Binary search for joint '%s' completed in %d iterations. Final value: %f (original: %f, direction: %s)",
              serch_joint_name_.c_str(), iteration, target_joint_values.joint_values[joint_idx], original, (direction > 0) ? "upper" : "lower");  

  return true;
}

bool PrimitiveExcavatorChangePosePlan::call_excavator_action_sync(
  const TmsRpExcavator::Goal& goal,
  TmsRpExcavator::Result::SharedPtr& result)
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

void PrimitiveExcavatorChangePosePlan::level_bucket_if_trigger(
  tms_msg_rp::msg::TmsRpExcavatorJointValues& jv,
  double trigger,
  double offset)
{
  int ib=-1, ia=-1, ik=-1;

  for (size_t i=0;i<jv.joint_names.size();++i) {
    if (jv.joint_names[i]=="boom_joint")   ib=i;
    if (jv.joint_names[i]=="arm_joint")    ia=i;
    if (jv.joint_names[i]=="bucket_joint") ik=i;
  }

  if (ib<0 || ia<0 || ik<0) return;
  if (std::abs(jv.joint_values[ik] - trigger) > 1e-9) return;

  jv.joint_values[ik] =
      -(jv.joint_values[ib] + jv.joint_values[ia]) + offset;
}

int main(int argc, char* argv[])
{
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrimitiveExcavatorChangePosePlan>());
  rclcpp::shutdown();
  return 0;
}
