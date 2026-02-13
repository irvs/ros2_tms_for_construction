// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
// Licensed under the Apache License, Version 2.0

#include "tms_ts_primitive/Excavator/primitive_excavator_change_pose_plan.hpp"
#include "tms_ts_primitive/Excavator/lib/excavator_pose_converter.hpp"
#include <glog/logging.h>
#include <bsoncxx/json.hpp>

using namespace std::chrono_literals;

// Helper function to get numeric value from BSON element
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

  auto goal_msg = TmsRpExcavator::Goal();
  goal_msg.previous_pose.clear();
  
  // Parse previous plan if specified
  if (!parse_previous_plan(goal_msg))
  {
    handle_error("Failed to parse previous plan");
    return;
  }
  
  // waypoints形式のみサポート
  if (!param_from_db_.count("waypoints")) {
    handle_error("waypoints field not found in DB. Please use waypoints format.");
    return;
  }

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

    ExcavatorPoseConverter pose_converter;
    
    // waypointsの数で処理を分岐
    if (num_waypoints == 1) {
      // ========== 1個の場合: CMD_PLAN_TO_JOINTS または CMD_PLAN_TO_POSE ==========
      auto waypoint_element = *waypoints_array.begin();
      auto waypoint_doc = waypoint_element.get_document().value;
      
      if (!waypoint_doc["type"]) {
        handle_error("Waypoint missing 'type' field");
        return;
      }
      
      std::string type = waypoint_doc["type"].get_string().value.to_string();
      
      if (type == "joint_values") {
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
        
        for (auto&& field : data_doc) {
          std::string joint_name = field.key().to_string();
          double joint_value = get_numeric_value(field);
          
          target_joint_values.joint_names.push_back(joint_name);
          target_joint_values.joint_values.push_back(joint_value);
        }
        
        goal_msg.joint_values_sequence.push_back(target_joint_values);
        RCLCPP_INFO(this->get_logger(), "  Target: %zu joints specified", target_joint_values.joint_names.size());
        
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
        
        goal_msg.pose_sequence.push_back(target_pose);
        RCLCPP_INFO(this->get_logger(), "  Target pose: (%.2f, %.2f, %.2f)", x, y, z);
        
      } else {
        handle_error("Unknown waypoint type: " + type);
        return;
      }
      
    } else {
      // ========== 2個以上の場合: CMD_PLAN_MOTION_SEQUENCE ==========
      goal_msg.command = TmsRpExcavator::Goal::CMD_PLAN_MOTION_SEQUENCE;
      RCLCPP_INFO(this->get_logger(), "===================================================");
      RCLCPP_INFO(this->get_logger(), "  Waypoints: %zu (Multiple waypoints)", num_waypoints);
      RCLCPP_INFO(this->get_logger(), "  Command: CMD_PLAN_MOTION_SEQUENCE");
      RCLCPP_INFO(this->get_logger(), "  Blend radius: 0.05 rad (~2.9 deg)");
      RCLCPP_INFO(this->get_logger(), "===================================================");
      
      goal_msg.motion_sequence_items.clear();
      
      size_t waypoint_index = 0;
      for (auto&& waypoint_element : waypoints_array) {
        if (waypoint_element.type() != bsoncxx::type::k_document) continue;
        
        auto waypoint_doc = waypoint_element.get_document().value;
        if (!waypoint_doc["type"]) continue;
        
        std::string type = waypoint_doc["type"].get_string().value.to_string();
        moveit_msgs::msg::MotionSequenceItem item;
        item.req.group_name = planning_group_;
        item.req.max_velocity_scaling_factor = 1.0;
        item.req.max_acceleration_scaling_factor = 1.0;
        item.req.allowed_planning_time = 5.0;
        item.req.planner_id = "PTP";
        item.req.pipeline_id = "pilz_industrial_motion_planner";
        
        if (type == "joint_values") {
          if (!waypoint_doc["data"]) continue;
          auto data_doc = waypoint_doc["data"].get_document().value;
          
          moveit_msgs::msg::Constraints constraints;
          
          for (auto&& field : data_doc) {
            std::string joint_name = field.key().to_string();
            double joint_value = get_numeric_value(field);
            
            moveit_msgs::msg::JointConstraint joint_constraint;
            joint_constraint.joint_name = joint_name;
            joint_constraint.position = joint_value;
            joint_constraint.tolerance_above = 0.01;
            joint_constraint.tolerance_below = 0.01;
            joint_constraint.weight = 1.0;
            constraints.joint_constraints.push_back(joint_constraint);
          }
          
          item.req.goal_constraints.push_back(constraints);
          RCLCPP_INFO(this->get_logger(), "  [%zu] joint_values (%zu joints)", 
                      waypoint_index + 1, constraints.joint_constraints.size());
          
        } else if (type == "pose") {
          if (!waypoint_doc["data"]) continue;
          auto data_doc = waypoint_doc["data"].get_document().value;
          
          if (!data_doc["x"] || !data_doc["y"] || !data_doc["z"] || !data_doc["theta_w"]) continue;
          
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
          
          moveit_msgs::msg::Constraints constraints;
          
          // Position constraint
          moveit_msgs::msg::PositionConstraint position_constraint;
          position_constraint.header.frame_id = "base_link";
          position_constraint.link_name = "bucket_end_link";
          
          shape_msgs::msg::SolidPrimitive primitive;
          primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
          primitive.dimensions.push_back(0.001);
          position_constraint.constraint_region.primitives.push_back(primitive);
          position_constraint.constraint_region.primitive_poses.push_back(target_pose);
          position_constraint.weight = 1.0;
          constraints.position_constraints.push_back(position_constraint);
          
          // Orientation constraint
          moveit_msgs::msg::OrientationConstraint orientation_constraint;
          orientation_constraint.header.frame_id = "base_link";
          orientation_constraint.link_name = "bucket_end_link";
          orientation_constraint.orientation = target_pose.orientation;
          orientation_constraint.absolute_x_axis_tolerance = 0.01;
          orientation_constraint.absolute_y_axis_tolerance = 0.01;
          orientation_constraint.absolute_z_axis_tolerance = 0.01;
          orientation_constraint.weight = 1.0;
          constraints.orientation_constraints.push_back(orientation_constraint);
          
          item.req.goal_constraints.push_back(constraints);
          RCLCPP_INFO(this->get_logger(), "  [%zu] pose (%.2f, %.2f, %.2f)", 
                      waypoint_index + 1, x, y, z);
        }
        
        // Blend radius設定（最後以外）
        if (waypoint_index < num_waypoints - 1) {
          item.blend_radius = 0.01;
        } else {
          item.blend_radius = 0.0;
        }
        
        goal_msg.motion_sequence_items.push_back(item);
        waypoint_index++;
      }
      
      RCLCPP_INFO(this->get_logger(), "Created %zu motion sequence items", 
                  goal_msg.motion_sequence_items.size());
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

  // Parse collision avoidance
  if (!parse_collision_avoidance(goal_msg))
  {
    handle_error("Failed to parse collision avoidance");
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
    if (result->plan.empty()) {
      RCLCPP_WARN(this->get_logger(), "Plan is empty, nothing to save");
      return;
    }
    
    bsoncxx::builder::basic::document plan_doc;
    int index = 1;
    
    for (const auto& robot_trajectory : result->plan) {
      bsoncxx::builder::basic::document trajectory_doc;
      
      if (!robot_trajectory.joint_trajectory.joint_names.empty()) {
        bsoncxx::builder::basic::document joint_traj_doc;
        
        bsoncxx::builder::basic::array joint_names_array;
        for (const auto& name : robot_trajectory.joint_trajectory.joint_names) {
          joint_names_array.append(name);
        }
        joint_traj_doc.append(bsoncxx::builder::basic::kvp("joint_names", joint_names_array));
        
        bsoncxx::builder::basic::array points_array;
        for (const auto& point : robot_trajectory.joint_trajectory.points) {
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
        trajectory_doc.append(bsoncxx::builder::basic::kvp("joint_trajectory", joint_traj_doc));
      }
      
      plan_doc.append(bsoncxx::builder::basic::kvp(std::to_string(index), trajectory_doc));
      index++;
    }
    
    std::string plan_json = bsoncxx::to_json(plan_doc.view());
    
    if(UpdateParamInDBFromJson(used_model_name_, used_record_name_, "plan", plan_json))
    {
      RCLCPP_INFO(this->get_logger(), "Successfully saved %zu plan(s) to database", result->plan.size());
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
  // 元コードの条件をそのまま
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

    auto plan_doc = bsoncxx::from_json(previous_param_from_db_["plan"]);
    auto plan_view = plan_doc.view();

    for (auto&& element : plan_view)
    {
      std::string key = element.key().to_string();
      try
      {
        (void)std::stoi(key); // 数字キー以外スキップするための判定

        auto trajectory_doc = element.get_document().value;
        moveit_msgs::msg::RobotTrajectory robot_trajectory;

        // joint_trajectory
        if (trajectory_doc["joint_trajectory"])
        {
          auto joint_traj_doc = trajectory_doc["joint_trajectory"].get_document().value;

          if (joint_traj_doc["joint_names"])
          {
            auto joint_names_array = joint_traj_doc["joint_names"].get_array().value;
            for (auto&& name : joint_names_array)
              robot_trajectory.joint_trajectory.joint_names.push_back(name.get_string().value.to_string());
          }

          if (joint_traj_doc["points"])
          {
            auto points_array = joint_traj_doc["points"].get_array().value;
            for (auto&& point_element : points_array)
            {
              auto point_doc = point_element.get_document().value;
              trajectory_msgs::msg::JointTrajectoryPoint point;

              if (point_doc["positions"])
              {
                auto arr = point_doc["positions"].get_array().value;
                for (auto&& v : arr) point.positions.push_back(get_numeric_value(v));
              }

              if (point_doc["velocities"])
              {
                auto arr = point_doc["velocities"].get_array().value;
                for (auto&& v : arr) point.velocities.push_back(get_numeric_value(v));
              }

              if (point_doc["accelerations"])
              {
                auto arr = point_doc["accelerations"].get_array().value;
                for (auto&& v : arr) point.accelerations.push_back(get_numeric_value(v));
              }

              if (point_doc["time_from_start"])
              {
                auto time_doc = point_doc["time_from_start"].get_document().value;
                if (time_doc["sec"])     point.time_from_start.sec     = time_doc["sec"].get_int32().value;
                if (time_doc["nanosec"]) point.time_from_start.nanosec = time_doc["nanosec"].get_int32().value;
              }

              robot_trajectory.joint_trajectory.points.push_back(point);
            }
          }
        }

        // multi_dof_joint_trajectory
        if (trajectory_doc["multi_dof_joint_trajectory"])
        {
          auto multi_dof_doc = trajectory_doc["multi_dof_joint_trajectory"].get_document().value;

          if (multi_dof_doc["joint_names"])
          {
            auto arr = multi_dof_doc["joint_names"].get_array().value;
            for (auto&& name : arr)
              robot_trajectory.multi_dof_joint_trajectory.joint_names.push_back(name.get_string().value.to_string());
          }

          if (multi_dof_doc["points"])
          {
            auto points_array = multi_dof_doc["points"].get_array().value;
            for (auto&& point_element : points_array)
            {
              auto point_doc = point_element.get_document().value;
              trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;

              if (point_doc["transforms"])
              {
                auto arr = point_doc["transforms"].get_array().value;
                for (auto&& t : arr)
                {
                  auto td = t.get_document().value;
                  geometry_msgs::msg::Transform transform;

                  if (td["translation"])
                  {
                    auto tr = td["translation"].get_document().value;
                    if (tr["x"]) transform.translation.x = get_numeric_value(tr["x"]);
                    if (tr["y"]) transform.translation.y = get_numeric_value(tr["y"]);
                    if (tr["z"]) transform.translation.z = get_numeric_value(tr["z"]);
                  }

                  if (td["rotation"])
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

              if (point_doc["velocities"])
              {
                auto arr = point_doc["velocities"].get_array().value;
                for (auto&& v : arr)
                {
                  auto vd = v.get_document().value;
                  geometry_msgs::msg::Twist twist;

                  if (vd["linear"])
                  {
                    auto li = vd["linear"].get_document().value;
                    if (li["x"]) twist.linear.x = get_numeric_value(li["x"]);
                    if (li["y"]) twist.linear.y = get_numeric_value(li["y"]);
                    if (li["z"]) twist.linear.z = get_numeric_value(li["z"]);
                  }
                  if (vd["angular"])
                  {
                    auto an = vd["angular"].get_document().value;
                    if (an["x"]) twist.angular.x = get_numeric_value(an["x"]);
                    if (an["y"]) twist.angular.y = get_numeric_value(an["y"]);
                    if (an["z"]) twist.angular.z = get_numeric_value(an["z"]);
                  }

                  point.velocities.push_back(twist);
                }
              }

              if (point_doc["accelerations"])
              {
                auto arr = point_doc["accelerations"].get_array().value;
                for (auto&& a : arr)
                {
                  auto ad = a.get_document().value;
                  geometry_msgs::msg::Twist twist;

                  if (ad["linear"])
                  {
                    auto li = ad["linear"].get_document().value;
                    if (li["x"]) twist.linear.x = get_numeric_value(li["x"]);
                    if (li["y"]) twist.linear.y = get_numeric_value(li["y"]);
                    if (li["z"]) twist.linear.z = get_numeric_value(li["z"]);
                  }
                  if (ad["angular"])
                  {
                    auto an = ad["angular"].get_document().value;
                    if (an["x"]) twist.angular.x = get_numeric_value(an["x"]);
                    if (an["y"]) twist.angular.y = get_numeric_value(an["y"]);
                    if (an["z"]) twist.angular.z = get_numeric_value(an["z"]);
                  }

                  point.accelerations.push_back(twist);
                }
              }

              if (point_doc["time_from_start"])
              {
                auto time_doc = point_doc["time_from_start"].get_document().value;
                if (time_doc["sec"])     point.time_from_start.sec     = time_doc["sec"].get_int32().value;
                if (time_doc["nanosec"]) point.time_from_start.nanosec = time_doc["nanosec"].get_int32().value;
              }

              robot_trajectory.multi_dof_joint_trajectory.points.push_back(point);
            }
          }
        }

        goal_msg.previous_pose.push_back(robot_trajectory);
      }
      catch (...)
      {
        continue;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu trajectories from previous plan", goal_msg.previous_pose.size());
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
        if (jc_doc["position"])
          joint_constraint.position = get_numeric_value(jc_doc["position"]);
        if (jc_doc["tolerance_above"])
          joint_constraint.tolerance_above = get_numeric_value(jc_doc["tolerance_above"]);
        if (jc_doc["tolerance_below"])
          joint_constraint.tolerance_below = get_numeric_value(jc_doc["tolerance_below"]);
        // ★元コードのバグ修正: weight を tolerance_below に入れていたので weight に入れる
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

bool PrimitiveExcavatorChangePosePlan::parse_collision_avoidance(TmsRpExcavator::Goal& goal_msg)
{
  try
  {
    if (!param_from_db_.count("collision_avoidance")) return true;

    auto doc = bsoncxx::from_json(param_from_db_["collision_avoidance"]);
    auto view = doc.view();
    if (!view["collision_avoidance"]) return true;

    RCLCPP_INFO(this->get_logger(), "Parsing collision_avoidance from JSON");
    auto collision_avoidance_doc = view["collision_avoidance"].get_document().value;

    std::string base_frame_id = "base_link";

    if (collision_avoidance_doc["constant"])
    {
      auto constant_doc = collision_avoidance_doc["constant"].get_document().value;
      moveit_msgs::msg::PlanningScene planning_scene;

      // primitives
      if (constant_doc["primitives"])
      {
        auto primitives_array = constant_doc["primitives"].get_array().value;
        for (auto&& prim : primitives_array)
        {
          auto prim_doc = prim.get_document().value;
          moveit_msgs::msg::CollisionObject collision_object;

          if (prim_doc["id"])
            collision_object.id = prim_doc["id"].get_string().value.to_string();

          collision_object.header.frame_id = base_frame_id;

          if (prim_doc["primitive"])
          {
            auto primitive_doc = prim_doc["primitive"].get_document().value;
            shape_msgs::msg::SolidPrimitive solid_primitive;

            if (primitive_doc["type"])
            {
              std::string type_str = primitive_doc["type"].get_string().value.to_string();
              if (type_str == "box") solid_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
              else if (type_str == "sphere") solid_primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
              else if (type_str == "cylinder") solid_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
              else if (type_str == "cone") solid_primitive.type = shape_msgs::msg::SolidPrimitive::CONE;
            }

            if (primitive_doc["dimensions"])
            {
              auto dimensions_array = primitive_doc["dimensions"].get_array().value;
              for (auto&& dim : dimensions_array)
                solid_primitive.dimensions.push_back(get_numeric_value(dim));
            }

            collision_object.primitives.push_back(solid_primitive);
          }

          if (prim_doc["pose"])
          {
            auto pose_doc = prim_doc["pose"].get_document().value;
            geometry_msgs::msg::Pose pose;

            if (pose_doc["position"])
            {
              auto pos = pose_doc["position"].get_document().value;
              if (pos["x"]) pose.position.x = get_numeric_value(pos["x"]);
              if (pos["y"]) pose.position.y = get_numeric_value(pos["y"]);
              if (pos["z"]) pose.position.z = get_numeric_value(pos["z"]);
            }

            if (pose_doc["orientation"])
            {
              auto ori = pose_doc["orientation"].get_document().value;
              if (ori["x"]) pose.orientation.x = get_numeric_value(ori["x"]);
              if (ori["y"]) pose.orientation.y = get_numeric_value(ori["y"]);
              if (ori["z"]) pose.orientation.z = get_numeric_value(ori["z"]);
              if (ori["w"]) pose.orientation.w = get_numeric_value(ori["w"]);
            }

            collision_object.primitive_poses.push_back(pose);
          }

          collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
          planning_scene.world.collision_objects.push_back(collision_object);
        }
      }

      // planes
      if (constant_doc["planes"])
      {
        auto planes_array = constant_doc["planes"].get_array().value;
        for (auto&& plane : planes_array)
        {
          auto plane_doc = plane.get_document().value;
          moveit_msgs::msg::CollisionObject collision_object;

          if (plane_doc["id"])
            collision_object.id = plane_doc["id"].get_string().value.to_string();

          collision_object.header.frame_id = base_frame_id;

          if (plane_doc["plane"])
          {
            auto plane_info = plane_doc["plane"].get_document().value;
            shape_msgs::msg::Plane plane_shape;

            if (plane_info["coef"])
            {
              auto coef_array = plane_info["coef"].get_array().value;
              auto it = coef_array.begin();
              if (it != coef_array.end()) plane_shape.coef[0] = get_numeric_value(*it++);
              if (it != coef_array.end()) plane_shape.coef[1] = get_numeric_value(*it++);
              if (it != coef_array.end()) plane_shape.coef[2] = get_numeric_value(*it++);
              if (it != coef_array.end()) plane_shape.coef[3] = get_numeric_value(*it++);
            }

            collision_object.planes.push_back(plane_shape);
          }

          if (plane_doc["pose"])
          {
            auto pose_doc = plane_doc["pose"].get_document().value;
            geometry_msgs::msg::Pose pose;

            if (pose_doc["position"])
            {
              auto pos = pose_doc["position"].get_document().value;
              if (pos["x"]) pose.position.x = get_numeric_value(pos["x"]);
              if (pos["y"]) pose.position.y = get_numeric_value(pos["y"]);
              if (pos["z"]) pose.position.z = get_numeric_value(pos["z"]);
            }

            if (pose_doc["orientation"])
            {
              auto ori = pose_doc["orientation"].get_document().value;
              if (ori["x"]) pose.orientation.x = get_numeric_value(ori["x"]);
              if (ori["y"]) pose.orientation.y = get_numeric_value(ori["y"]);
              if (ori["z"]) pose.orientation.z = get_numeric_value(ori["z"]);
              if (ori["w"]) pose.orientation.w = get_numeric_value(ori["w"]);
            }

            collision_object.plane_poses.push_back(pose);
          }

          collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
          planning_scene.world.collision_objects.push_back(collision_object);
        }
      }

      goal_msg.planning_scene = planning_scene;
      RCLCPP_INFO(this->get_logger(), "Added %zu collision objects to planning scene",
                  planning_scene.world.collision_objects.size());
    }

    // link_padding
    if (collision_avoidance_doc["link_padding"])
    {
      RCLCPP_INFO(this->get_logger(), "Parsing link_padding from JSON");
      auto link_padding_doc = collision_avoidance_doc["link_padding"].get_document().value;

      for (auto&& element : link_padding_doc)
      {
        std::string link_name = element.key().to_string();
        double padding_value = get_numeric_value(element);

        moveit_msgs::msg::LinkPadding link_padding;
        link_padding.link_name = link_name;
        link_padding.padding = padding_value;

        goal_msg.planning_scene.link_padding.push_back(link_padding);
      }

      RCLCPP_INFO(this->get_logger(), "Added %zu link padding entries",
                  goal_msg.planning_scene.link_padding.size());
    }

    goal_msg.planning_scene.is_diff = true;
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse collision_avoidance: %s", e.what());
    return false;
  }
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
