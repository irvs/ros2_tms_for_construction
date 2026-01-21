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

#include "tms_ts_primitive/Excavator/primitive_excavator_change_pose_plan_from_poses.hpp"
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

PrimitiveExcavatorChangePoseFromPose::PrimitiveExcavatorChangePoseFromPose() : PrimitiveNodeBase("primitive_excavator_change_pose_from_pose_node")
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
      this, "primitive_excavator_change_pose_from_poses",
      std::bind(&PrimitiveExcavatorChangePoseFromPose::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PrimitiveExcavatorChangePoseFromPose::handle_cancel, this, std::placeholders::_1),
      std::bind(&PrimitiveExcavatorChangePoseFromPose::handle_accepted, this, std::placeholders::_1),
      options_server);

  action_client_ = rclcpp_action::create_client<ExcavatorChangePoseFromPose>(this, "tms_rp_excavator_change_pose_plan_from_poses",nullptr ,options_client);
  if (action_client_->wait_for_action_server())
  {
    RCLCPP_INFO(this->get_logger(), "Action server is ready");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
  }
}

rclcpp_action::GoalResponse PrimitiveExcavatorChangePoseFromPose::handle_goal(
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
  else
  {
    RCLCPP_INFO(this->get_logger(), "No previous plan specified, will plan from current pose");
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PrimitiveExcavatorChangePoseFromPose::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
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

void PrimitiveExcavatorChangePoseFromPose::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
  using namespace std::placeholders;
  std::thread{ std::bind(&PrimitiveExcavatorChangePoseFromPose::execute, this, _1), goal_handle }.detach();
}

void PrimitiveExcavatorChangePoseFromPose::execute(const std::shared_ptr<GoalHandle> goal_handle)
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
  if (!action_client_->action_server_is_ready())
  {
    handle_error("Action server not available");
    return;
  }

  auto goal_msg = ExcavatorChangePoseFromPose::Goal();
  goal_msg.joint_values_sequence.clear();
  goal_msg.previous_pose.clear();

  RCLCPP_INFO(this->get_logger(), "Get pose from DB.");

  // RCLCPP_INFO(this->get_logger(), "param_from_db_ contents:");
  // for (const auto& [key, value] : param_from_db_)
  // {
  //   RCLCPP_INFO(this->get_logger(), "  %s: %s", key.c_str(), value.c_str());
  // }
  // for (const auto& [key, value] : previous_param_from_db_)
  // {
  //   RCLCPP_INFO(this->get_logger(), "  %s: %s", key.c_str(), value.c_str());
  // }
  
  // previous_target_record_nameが空でない場合、データベースからplanを取得してRobotTrajectory配列に変換
  if (!previous_target_record_name_.empty() && !previous_param_from_db_.empty())
  {
    RCLCPP_INFO(this->get_logger(), "Loading previous plan from record: %s", previous_target_record_name_.c_str());
    
    try {
      if (previous_param_from_db_.count("plan")) {
        auto plan_doc = bsoncxx::from_json(previous_param_from_db_["plan"]);
        auto plan_view = plan_doc.view();
        
        // planの各trajectory("1", "2", "3"...)を処理
        for (auto&& element : plan_view) {
          std::string key = element.key().to_string();
          
          // 数字のキーのみ処理
          try {
            int index = std::stoi(key);
            
            auto trajectory_doc = element.get_document().value;
            moveit_msgs::msg::RobotTrajectory robot_trajectory;
            
            // joint_trajectoryの変換
            if (trajectory_doc["joint_trajectory"]) {
              auto joint_traj_doc = trajectory_doc["joint_trajectory"].get_document().value;
              
              // joint_names
              if (joint_traj_doc["joint_names"]) {
                auto joint_names_array = joint_traj_doc["joint_names"].get_array().value;
                for (auto&& name : joint_names_array) {
                  robot_trajectory.joint_trajectory.joint_names.push_back(name.get_string().value.to_string());
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
                      point.time_from_start.nanosec = time_doc["nanosec"].get_int32().value;
                    }
                  }
                  
                  robot_trajectory.joint_trajectory.points.push_back(point);
                }
              }
            }
            
            // multi_dof_joint_trajectoryの変換（もし存在すれば）
            if (trajectory_doc["multi_dof_joint_trajectory"]) {
              auto multi_dof_doc = trajectory_doc["multi_dof_joint_trajectory"].get_document().value;
              
              // joint_names
              if (multi_dof_doc["joint_names"]) {
                auto joint_names_array = multi_dof_doc["joint_names"].get_array().value;
                for (auto&& name : joint_names_array) {
                  robot_trajectory.multi_dof_joint_trajectory.joint_names.push_back(name.get_string().value.to_string());
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
                      
                      if (transform_doc["translation"]) {
                        auto translation = transform_doc["translation"].get_document().value;
                        if (translation["x"]) transform.translation.x = get_numeric_value(translation["x"]);
                        if (translation["y"]) transform.translation.y = get_numeric_value(translation["y"]);
                        if (translation["z"]) transform.translation.z = get_numeric_value(translation["z"]);
                      }
                      
                      if (transform_doc["rotation"]) {
                        auto rotation = transform_doc["rotation"].get_document().value;
                        if (rotation["x"]) transform.rotation.x = get_numeric_value(rotation["x"]);
                        if (rotation["y"]) transform.rotation.y = get_numeric_value(rotation["y"]);
                        if (rotation["z"]) transform.rotation.z = get_numeric_value(rotation["z"]);
                        if (rotation["w"]) transform.rotation.w = get_numeric_value(rotation["w"]);
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
                      
                      if (vel_doc["linear"]) {
                        auto linear = vel_doc["linear"].get_document().value;
                        if (linear["x"]) twist.linear.x = get_numeric_value(linear["x"]);
                        if (linear["y"]) twist.linear.y = get_numeric_value(linear["y"]);
                        if (linear["z"]) twist.linear.z = get_numeric_value(linear["z"]);
                      }
                      
                      if (vel_doc["angular"]) {
                        auto angular = vel_doc["angular"].get_document().value;
                        if (angular["x"]) twist.angular.x = get_numeric_value(angular["x"]);
                        if (angular["y"]) twist.angular.y = get_numeric_value(angular["y"]);
                        if (angular["z"]) twist.angular.z = get_numeric_value(angular["z"]);
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
                      
                      if (acc_doc["linear"]) {
                        auto linear = acc_doc["linear"].get_document().value;
                        if (linear["x"]) twist.linear.x = get_numeric_value(linear["x"]);
                        if (linear["y"]) twist.linear.y = get_numeric_value(linear["y"]);
                        if (linear["z"]) twist.linear.z = get_numeric_value(linear["z"]);
                      }
                      
                      if (acc_doc["angular"]) {
                        auto angular = acc_doc["angular"].get_document().value;
                        if (angular["x"]) twist.angular.x = get_numeric_value(angular["x"]);
                        if (angular["y"]) twist.angular.y = get_numeric_value(angular["y"]);
                        if (angular["z"]) twist.angular.z = get_numeric_value(angular["z"]);
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
                      point.time_from_start.nanosec = time_doc["nanosec"].get_int32().value;
                    }
                  }
                  
                  robot_trajectory.multi_dof_joint_trajectory.points.push_back(point);
                }
              }
            }
            
            goal_msg.previous_pose.push_back(robot_trajectory);
            RCLCPP_INFO(this->get_logger(), "Loaded trajectory %d from previous plan", index);
          } catch (...) {
            // 数字でないキーはスキップ
            continue;
          }
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu trajectories from previous plan", goal_msg.previous_pose.size());
      } else {
        RCLCPP_INFO(this->get_logger(), "Previous record has no plan, will plan from current pose");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse previous plan: %s. Will plan from current pose", e.what());
    }
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "No previous plan specified, will plan from current pose");
  }

  // JSON文字列からBSONドキュメントに変換してメッセージ型に設定
  try {
    // position_with_angleパラメータのチェック
    if (!param_from_db_.count("position_with_angle")) {
      RCLCPP_ERROR(this->get_logger(), "Missing required parameter: position_with_angle");
      handle_error("Missing required parameter: position_with_angle");
      return;
    }

    // position_with_angle配列のパース
    auto doc = bsoncxx::from_json(param_from_db_["position_with_angle"]);
    auto view = doc.view();
    
    if (!view["position_with_angle"]) {
      RCLCPP_ERROR(this->get_logger(), "position_with_angle field not found in JSON");
      handle_error("position_with_angle field not found");
      return;
    }
    
    auto position_with_angle_element = view["position_with_angle"];
    
    if (position_with_angle_element.type() != bsoncxx::type::k_array) {
      RCLCPP_ERROR(this->get_logger(), "position_with_angle must be an array");
      handle_error("position_with_angle must be an array");
      return;
    }
    
    auto position_array = position_with_angle_element.get_array().value;
    
    // 各位置データを処理
    for (auto&& pos_element : position_array) {
      if (pos_element.type() != bsoncxx::type::k_document) {
        RCLCPP_ERROR(this->get_logger(), "Each element in position_with_angle must be a document");
        continue;
      }
      
      auto pos_doc = pos_element.get_document().value;
      tms_msg_rp::msg::TmsRpExcavatorPositionWithAngle target_pose;
      
      // x, y, z, theta_wの抽出
      if (!pos_doc["x"] || !pos_doc["y"] || !pos_doc["z"] || !pos_doc["theta_w"]) {
        RCLCPP_ERROR(this->get_logger(), "Missing required fields (x, y, z, theta_w) in position element");
        continue;
      }
      
      target_pose.position.x = get_numeric_value(pos_doc["x"]);
      target_pose.position.y = get_numeric_value(pos_doc["y"]);
      target_pose.position.z = get_numeric_value(pos_doc["z"]);
      target_pose.theta_w = get_numeric_value(pos_doc["theta_w"]);
      
      goal_msg.position_with_angle_sequence.push_back(target_pose);
      RCLCPP_INFO(this->get_logger(), "Added target pose: x=%f, y=%f, z=%f, theta_w=%f", 
                  target_pose.position.x, target_pose.position.y, target_pose.position.z, target_pose.theta_w);
    }
    
    if (goal_msg.position_with_angle_sequence.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No valid positions were added to the sequence");
      handle_error("No valid positions in position_with_angle array");
      return;
    }

    // constraintsの設定
    if (param_from_db_.count("constraints")) {
      auto doc = bsoncxx::from_json(param_from_db_["constraints"]);
      auto view = doc.view();
      
      if (view["constraints"]) {
        RCLCPP_INFO(this->get_logger(), "Parsing constraints from JSON");
        auto constraints_doc = view["constraints"].get_document().value;
        
        // joint_constraintsの変換
        if (constraints_doc["joint_constraints"]) {
          auto joint_constraints = constraints_doc["joint_constraints"].get_array().value;
          for (auto&& jc : joint_constraints) {
            auto jc_doc = jc.get_document().value;
            moveit_msgs::msg::JointConstraint joint_constraint;
            
            if (jc_doc["joint_name"]) joint_constraint.joint_name = jc_doc["joint_name"].get_string().value.to_string();
            if (jc_doc["position"]) joint_constraint.position = get_numeric_value(jc_doc["position"]);
            if (jc_doc["tolerance_above"]) joint_constraint.tolerance_above = get_numeric_value(jc_doc["tolerance_above"]);
            if (jc_doc["tolerance_below"]) joint_constraint.tolerance_below = get_numeric_value(jc_doc["tolerance_below"]);
            if (jc_doc["weight"]) joint_constraint.tolerance_below = get_numeric_value(jc_doc["weight"]);
            
            goal_msg.constraints.joint_constraints.push_back(joint_constraint);
          }
          RCLCPP_INFO(this->get_logger(), "Added %zu joint constraints", goal_msg.constraints.joint_constraints.size());
        }
        
        // position_constraintsの変換
        if (constraints_doc["position_constraints"]) {
          auto position_constraints = constraints_doc["position_constraints"].get_array().value;
          for (auto&& pc : position_constraints) {
            auto pc_doc = pc.get_document().value;
            moveit_msgs::msg::PositionConstraint position_constraint;
            
            // header
            if (pc_doc["header"] && pc_doc["header"].get_document().value["frame_id"]) {
              position_constraint.header.frame_id = pc_doc["header"].get_document().value["frame_id"].get_string().value.to_string();
            }
            
            // link_name
            if (pc_doc["link_name"]) position_constraint.link_name = pc_doc["link_name"].get_string().value.to_string();
            
            // target_point_offset
            if (pc_doc["target_point_offset"]) {
              auto offset = pc_doc["target_point_offset"].get_document().value;
              if (offset["x"]) position_constraint.target_point_offset.x = get_numeric_value(offset["x"]);
              if (offset["y"]) position_constraint.target_point_offset.y = get_numeric_value(offset["y"]);
              if (offset["z"]) position_constraint.target_point_offset.z = get_numeric_value(offset["z"]);
            }
            
            // constraint_region
            if (pc_doc["constraint_region"]) {
              auto region = pc_doc["constraint_region"].get_document().value;
              
              // primitives
              if (region["primitives"]) {
                auto primitives = region["primitives"].get_array().value;
                for (auto&& prim : primitives) {
                  auto prim_doc = prim.get_document().value;
                  shape_msgs::msg::SolidPrimitive solid_primitive;
                  
                  if (prim_doc["type"]) {
                    if (prim_doc["type"].type() == bsoncxx::type::k_int32) {
                      solid_primitive.type = prim_doc["type"].get_int32().value;
                    } else if (prim_doc["type"].type() == bsoncxx::type::k_int64) {
                      solid_primitive.type = static_cast<uint8_t>(prim_doc["type"].get_int64().value);
                    }
                  }
                  
                  if (prim_doc["dimensions"]) {
                    auto dimensions = prim_doc["dimensions"].get_array().value;
                    for (auto&& dim : dimensions) {
                      solid_primitive.dimensions.push_back(get_numeric_value(dim));
                    }
                  }
                  
                  position_constraint.constraint_region.primitives.push_back(solid_primitive);
                }
              }
              
              // primitive_poses
              if (region["primitive_poses"]) {
                auto poses = region["primitive_poses"].get_array().value;
                for (auto&& pose : poses) {
                  auto pose_doc = pose.get_document().value;
                  geometry_msgs::msg::Pose geo_pose;
                  
                  if (pose_doc["position"]) {
                    auto pos = pose_doc["position"].get_document().value;
                    if (pos["x"]) geo_pose.position.x = get_numeric_value(pos["x"]);
                    if (pos["y"]) geo_pose.position.y = get_numeric_value(pos["y"]);
                    if (pos["z"]) geo_pose.position.z = get_numeric_value(pos["z"]);
                  }
                  
                  if (pose_doc["orientation"]) {
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
            if (pc_doc["weight"]) {
              if (pc_doc["weight"].type() == bsoncxx::type::k_double) {
                position_constraint.weight = get_numeric_value(pc_doc["weight"]);
              }
            }
            
            goal_msg.constraints.position_constraints.push_back(position_constraint);
          }
          RCLCPP_INFO(this->get_logger(), "Added %zu position constraints", goal_msg.constraints.position_constraints.size());
        }
        
        // orientation_constraintsの変換
        if (constraints_doc["orientation_constraints"]) {
          auto orientation_constraints = constraints_doc["orientation_constraints"].get_array().value;
          for (auto&& oc : orientation_constraints) {
            auto oc_doc = oc.get_document().value;
            moveit_msgs::msg::OrientationConstraint orientation_constraint;
            
            // header
            if (oc_doc["header"] && oc_doc["header"].get_document().value["frame_id"]) {
              orientation_constraint.header.frame_id = oc_doc["header"].get_document().value["frame_id"].get_string().value.to_string();
            }
            
            // link_name
            if (oc_doc["link_name"]) orientation_constraint.link_name = oc_doc["link_name"].get_string().value.to_string();
            
            // orientation
            if (oc_doc["orientation"]) {
              auto ori = oc_doc["orientation"].get_document().value;
              if (ori["x"]) orientation_constraint.orientation.x = get_numeric_value(ori["x"]);
              if (ori["y"]) orientation_constraint.orientation.y = get_numeric_value(ori["y"]);
              if (ori["z"]) orientation_constraint.orientation.z = get_numeric_value(ori["z"]);
              if (ori["w"]) orientation_constraint.orientation.w = get_numeric_value(ori["w"]);
            }
            
            // tolerances
            if (oc_doc["absolute_x_axis_tolerance"]) orientation_constraint.absolute_x_axis_tolerance = get_numeric_value(oc_doc["absolute_x_axis_tolerance"]);
            if (oc_doc["absolute_y_axis_tolerance"]) orientation_constraint.absolute_y_axis_tolerance = get_numeric_value(oc_doc["absolute_y_axis_tolerance"]);
            if (oc_doc["absolute_z_axis_tolerance"]) orientation_constraint.absolute_z_axis_tolerance = get_numeric_value(oc_doc["absolute_z_axis_tolerance"]);
            
            // weight
            if (oc_doc["weight"]) {
              if (oc_doc["weight"].type() == bsoncxx::type::k_double) {
                orientation_constraint.weight = get_numeric_value(oc_doc["weight"]);
              }
            }
            
            goal_msg.constraints.orientation_constraints.push_back(orientation_constraint);
          }
          RCLCPP_INFO(this->get_logger(), "Added %zu orientation constraints", goal_msg.constraints.orientation_constraints.size());
        }
        
        // visibility_constraintsの変換
        if (constraints_doc["visibility_constraints"]) {
          auto visibility_constraints = constraints_doc["visibility_constraints"].get_array().value;
          for (auto&& vc : visibility_constraints) {
            auto vc_doc = vc.get_document().value;
            moveit_msgs::msg::VisibilityConstraint visibility_constraint;
            
            // target_radius
            if (vc_doc["target_radius"]) visibility_constraint.target_radius = get_numeric_value(vc_doc["target_radius"]);
            
            // target_pose
            if (vc_doc["target_pose"]) {
              auto target_pose = vc_doc["target_pose"].get_document().value;
              
              if (target_pose["header"] && target_pose["header"].get_document().value["frame_id"]) {
                visibility_constraint.target_pose.header.frame_id = target_pose["header"].get_document().value["frame_id"].get_string().value.to_string();
              }
              
              if (target_pose["pose"]) {
                auto pose = target_pose["pose"].get_document().value;
                if (pose["position"]) {
                  auto pos = pose["position"].get_document().value;
                  if (pos["x"]) visibility_constraint.target_pose.pose.position.x = get_numeric_value(pos["x"]);
                  if (pos["y"]) visibility_constraint.target_pose.pose.position.y = get_numeric_value(pos["y"]);
                  if (pos["z"]) visibility_constraint.target_pose.pose.position.z = get_numeric_value(pos["z"]);
                }
                if (pose["orientation"]) {
                  auto ori = pose["orientation"].get_document().value;
                  if (ori["x"]) visibility_constraint.target_pose.pose.orientation.x = get_numeric_value(ori["x"]);
                  if (ori["y"]) visibility_constraint.target_pose.pose.orientation.y = get_numeric_value(ori["y"]);
                  if (ori["z"]) visibility_constraint.target_pose.pose.orientation.z = get_numeric_value(ori["z"]);
                  if (ori["w"]) visibility_constraint.target_pose.pose.orientation.w = get_numeric_value(ori["w"]);
                }
              }
            }
            
            // cone_sides
            if (vc_doc["cone_sides"]) {
              if (vc_doc["cone_sides"].type() == bsoncxx::type::k_int32) {
                visibility_constraint.cone_sides = vc_doc["cone_sides"].get_int32().value;
              } else if (vc_doc["cone_sides"].type() == bsoncxx::type::k_int64) {
                visibility_constraint.cone_sides = static_cast<int32_t>(vc_doc["cone_sides"].get_int64().value);
              }
            }
            
            // sensor_pose
            if (vc_doc["sensor_pose"]) {
              auto sensor_pose = vc_doc["sensor_pose"].get_document().value;
              
              if (sensor_pose["header"] && sensor_pose["header"].get_document().value["frame_id"]) {
                visibility_constraint.sensor_pose.header.frame_id = sensor_pose["header"].get_document().value["frame_id"].get_string().value.to_string();
              }
              
              if (sensor_pose["pose"]) {
                auto pose = sensor_pose["pose"].get_document().value;
                if (pose["position"]) {
                  auto pos = pose["position"].get_document().value;
                  if (pos["x"]) visibility_constraint.sensor_pose.pose.position.x = get_numeric_value(pos["x"]);
                  if (pos["y"]) visibility_constraint.sensor_pose.pose.position.y = get_numeric_value(pos["y"]);
                  if (pos["z"]) visibility_constraint.sensor_pose.pose.position.z = get_numeric_value(pos["z"]);
                }
                if (pose["orientation"]) {
                  auto ori = pose["orientation"].get_document().value;
                  if (ori["x"]) visibility_constraint.sensor_pose.pose.orientation.x = get_numeric_value(ori["x"]);
                  if (ori["y"]) visibility_constraint.sensor_pose.pose.orientation.y = get_numeric_value(ori["y"]);
                  if (ori["z"]) visibility_constraint.sensor_pose.pose.orientation.z = get_numeric_value(ori["z"]);
                  if (ori["w"]) visibility_constraint.sensor_pose.pose.orientation.w = get_numeric_value(ori["w"]);
                }
              }
            }
            
            // angles
            if (vc_doc["max_view_angle"]) visibility_constraint.max_view_angle = get_numeric_value(vc_doc["max_view_angle"]);
            if (vc_doc["max_range_angle"]) visibility_constraint.max_range_angle = get_numeric_value(vc_doc["max_range_angle"]);
            
            // sensor_view_direction
            if (vc_doc["sensor_view_direction"]) {
              if (vc_doc["sensor_view_direction"].type() == bsoncxx::type::k_int32) {
                visibility_constraint.sensor_view_direction = vc_doc["sensor_view_direction"].get_int32().value;
              } else if (vc_doc["sensor_view_direction"].type() == bsoncxx::type::k_int64) {
                visibility_constraint.sensor_view_direction = static_cast<uint8_t>(vc_doc["sensor_view_direction"].get_int64().value);
              }
            }
            
            // weight
            if (vc_doc["weight"]) {
              if (vc_doc["weight"].type() == bsoncxx::type::k_double) {
                visibility_constraint.weight = get_numeric_value(vc_doc["weight"]);
              }
            }
            
            goal_msg.constraints.visibility_constraints.push_back(visibility_constraint);
          }
          RCLCPP_INFO(this->get_logger(), "Added %zu visibility constraints", goal_msg.constraints.visibility_constraints.size());
        }
      }
    }

    // collision_avoidanceの設定
    if (param_from_db_.count("collision_avoidance")) {
      auto doc = bsoncxx::from_json(param_from_db_["collision_avoidance"]);
      auto view = doc.view();
      
      if (view["collision_avoidance"]) {
        RCLCPP_INFO(this->get_logger(), "Parsing collision_avoidance from JSON");
        auto collision_avoidance_doc = view["collision_avoidance"].get_document().value;
        
        // model_nameの取得
        // std::string base_frame_id = used_model_name_ + "/base_link";
        std::string base_frame_id = "base_link";
        
        // constantの処理
        if (collision_avoidance_doc["constant"]) {
          auto constant_doc = collision_avoidance_doc["constant"].get_document().value;
          moveit_msgs::msg::PlanningScene planning_scene;
          
          // primitives の処理
          if (constant_doc["primitives"]) {
            auto primitives_array = constant_doc["primitives"].get_array().value;
            for (auto&& prim : primitives_array) {
              auto prim_doc = prim.get_document().value;
              moveit_msgs::msg::CollisionObject collision_object;
              
              // ID の設定
              if (prim_doc["id"]) {
                collision_object.id = prim_doc["id"].get_string().value.to_string();
              }
              
              // header の設定
              collision_object.header.frame_id = base_frame_id;
              
              // primitive の設定
              if (prim_doc["primitive"]) {
                auto primitive_doc = prim_doc["primitive"].get_document().value;
                shape_msgs::msg::SolidPrimitive solid_primitive;
                
                // type の設定
                if (primitive_doc["type"]) {
                  std::string type_str = primitive_doc["type"].get_string().value.to_string();
                  if (type_str == "box") {
                    solid_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
                  } else if (type_str == "sphere") {
                    solid_primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
                  } else if (type_str == "cylinder") {
                    solid_primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                  } else if (type_str == "cone") {
                    solid_primitive.type = shape_msgs::msg::SolidPrimitive::CONE;
                  }
                }
                
                // dimensions の設定
                if (primitive_doc["dimensions"]) {
                  auto dimensions_array = primitive_doc["dimensions"].get_array().value;
                  for (auto&& dim : dimensions_array) {
                    solid_primitive.dimensions.push_back(get_numeric_value(dim));
                  }
                }
                
                collision_object.primitives.push_back(solid_primitive);
              }
              
              // pose の設定
              if (prim_doc["pose"]) {
                auto pose_doc = prim_doc["pose"].get_document().value;
                geometry_msgs::msg::Pose pose;
                
                if (pose_doc["position"]) {
                  auto pos = pose_doc["position"].get_document().value;
                  if (pos["x"]) pose.position.x = get_numeric_value(pos["x"]);
                  if (pos["y"]) pose.position.y = get_numeric_value(pos["y"]);
                  if (pos["z"]) pose.position.z = get_numeric_value(pos["z"]);
                }
                
                if (pose_doc["orientation"]) {
                  auto ori = pose_doc["orientation"].get_document().value;
                  if (ori["x"]) pose.orientation.x = get_numeric_value(ori["x"]);
                  if (ori["y"]) pose.orientation.y = get_numeric_value(ori["y"]);
                  if (ori["z"]) pose.orientation.z = get_numeric_value(ori["z"]);
                  if (ori["w"]) pose.orientation.w = get_numeric_value(ori["w"]);
                }
                
                collision_object.primitive_poses.push_back(pose);
              }
              
              // operation の設定（追加）
              collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
              
              planning_scene.world.collision_objects.push_back(collision_object);
              RCLCPP_INFO(this->get_logger(), "Added collision object: %s", collision_object.id.c_str());
            }
          }
          
          // planes の処理
          if (constant_doc["planes"]) {
            auto planes_array = constant_doc["planes"].get_array().value;
            for (auto&& plane : planes_array) {
              auto plane_doc = plane.get_document().value;
              moveit_msgs::msg::CollisionObject collision_object;
              
              // ID の設定
              if (plane_doc["id"]) {
                collision_object.id = plane_doc["id"].get_string().value.to_string();
              }
              
              // header の設定
              collision_object.header.frame_id = base_frame_id;
              
              // plane の設定
              if (plane_doc["plane"]) {
                auto plane_info = plane_doc["plane"].get_document().value;
                shape_msgs::msg::Plane plane_shape;
                
                // coef の設定 (a, b, c, d)
                if (plane_info["coef"]) {
                  auto coef_array = plane_info["coef"].get_array().value;
                  auto it = coef_array.begin();
                  if (it != coef_array.end()) plane_shape.coef[0] = get_numeric_value(*it++);
                  if (it != coef_array.end()) plane_shape.coef[1] = get_numeric_value(*it++);
                  if (it != coef_array.end()) plane_shape.coef[2] = get_numeric_value(*it++);
                  if (it != coef_array.end()) plane_shape.coef[3] = get_numeric_value(*it++);
                }
                
                collision_object.planes.push_back(plane_shape);
              }
              
              // pose の設定
              if (plane_doc["pose"]) {
                auto pose_doc = plane_doc["pose"].get_document().value;
                geometry_msgs::msg::Pose pose;
                
                if (pose_doc["position"]) {
                  auto pos = pose_doc["position"].get_document().value;
                  if (pos["x"]) pose.position.x = get_numeric_value(pos["x"]);
                  if (pos["y"]) pose.position.y = get_numeric_value(pos["y"]);
                  if (pos["z"]) pose.position.z = get_numeric_value(pos["z"]);
                }
                
                if (pose_doc["orientation"]) {
                  auto ori = pose_doc["orientation"].get_document().value;
                  if (ori["x"]) pose.orientation.x = get_numeric_value(ori["x"]);
                  if (ori["y"]) pose.orientation.y = get_numeric_value(ori["y"]);
                  if (ori["z"]) pose.orientation.z = get_numeric_value(ori["z"]);
                  if (ori["w"]) pose.orientation.w = get_numeric_value(ori["w"]);
                }
                
                collision_object.plane_poses.push_back(pose);
              }
              
              // operation の設定（追加）
              collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
              
              planning_scene.world.collision_objects.push_back(collision_object);
              RCLCPP_INFO(this->get_logger(), "Added plane object: %s", collision_object.id.c_str());
            }
          }
          
          // Planning Sceneをgoal_msgに設定
          goal_msg.planning_scene = planning_scene;
          RCLCPP_INFO(this->get_logger(), "Added %zu collision objects to planning scene", 
                      planning_scene.world.collision_objects.size());
        }
        
        // link_paddingの処理
        if (collision_avoidance_doc["link_padding"]) {
          RCLCPP_INFO(this->get_logger(), "Parsing link_padding from JSON");
          auto link_padding_doc = collision_avoidance_doc["link_padding"].get_document().value;
          
          // 各リンク名とpadding値を取得
          for (auto&& element : link_padding_doc) {
            std::string link_name = element.key().to_string();
            double padding_value = get_numeric_value(element);
            
            moveit_msgs::msg::LinkPadding link_padding;
            link_padding.link_name = link_name;
            link_padding.padding = padding_value;
            
            goal_msg.planning_scene.link_padding.push_back(link_padding);
            RCLCPP_INFO(this->get_logger(), "Added link padding: %s = %f", 
                        link_name.c_str(), padding_value);
          }
          
          RCLCPP_INFO(this->get_logger(), "Added %zu link padding entries", 
                      goal_msg.planning_scene.link_padding.size());
        }
        goal_msg.planning_scene.is_diff = true;
      }
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse parameters: %s", e.what());
    handle_error("Failed to parse parameters from DB");
    return;
  }

  // Send goal to TMS_RP
  auto send_goal_options = rclcpp_action::Client<ExcavatorChangePoseFromPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = [this](const auto& goal_handle) { goal_response_callback(goal_handle); };
  send_goal_options.feedback_callback = [this](const auto tmp, const auto feedback) {
    feedback_callback(tmp, feedback);
  };
  send_goal_options.result_callback = [this, goal_handle](const auto& result) { result_callback(goal_handle, result); };

  RCLCPP_INFO(this->get_logger(), "Sending goal");

  client_future_goal_handle_ = action_client_->async_send_goal(goal_msg, send_goal_options);
}

void PrimitiveExcavatorChangePoseFromPose::goal_response_callback(const GoalHandleExcavatorChangePoseFromPose::SharedPtr& goal_handle)
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

void PrimitiveExcavatorChangePoseFromPose::feedback_callback(
    const GoalHandleExcavatorChangePoseFromPose::SharedPtr,
    const std::shared_ptr<const GoalHandleExcavatorChangePoseFromPose::Feedback> feedback)
{
  // TODO: Fix to feedback to leaf node
  RCLCPP_INFO(this->get_logger(), "Feedback received: %s", feedback->state.c_str());
}

void PrimitiveExcavatorChangePoseFromPose::result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                                             const GoalHandleExcavatorChangePoseFromPose::WrappedResult& result)
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
  
  // planをデータベースに保存
  try {
    // planが空でないことを確認
    if (result.result->plan.empty()) {
      RCLCPP_WARN(this->get_logger(), "Plan is empty, nothing to save");
      return;
    }
    
    // すべてのRobotTrajectoryをBSONドキュメント形式で保存 (キー: "1", "2", "3"...)
    bsoncxx::builder::basic::document plan_doc;
    
    int index = 1;
    for (const auto& robot_trajectory : result.result->plan) {
      bsoncxx::builder::basic::document trajectory_doc;
      
      // joint_trajectoryの変換
      if (!robot_trajectory.joint_trajectory.joint_names.empty()) {
        bsoncxx::builder::basic::document joint_traj_doc;
        
        // joint_names
        bsoncxx::builder::basic::array joint_names_array;
        for (const auto& name : robot_trajectory.joint_trajectory.joint_names) {
          joint_names_array.append(name);
        }
        joint_traj_doc.append(bsoncxx::builder::basic::kvp("joint_names", joint_names_array));
        
        // points
        bsoncxx::builder::basic::array points_array;
        for (const auto& point : robot_trajectory.joint_trajectory.points) {
          bsoncxx::builder::basic::document point_doc;
          
          // positions
          bsoncxx::builder::basic::array positions_array;
          for (const auto& pos : point.positions) {
            positions_array.append(pos);
          }
          point_doc.append(bsoncxx::builder::basic::kvp("positions", positions_array));
          
          // velocities
          if (!point.velocities.empty()) {
            bsoncxx::builder::basic::array velocities_array;
            for (const auto& vel : point.velocities) {
              velocities_array.append(vel);
            }
            point_doc.append(bsoncxx::builder::basic::kvp("velocities", velocities_array));
          }
          
          // accelerations
          if (!point.accelerations.empty()) {
            bsoncxx::builder::basic::array accelerations_array;
            for (const auto& acc : point.accelerations) {
              accelerations_array.append(acc);
            }
            point_doc.append(bsoncxx::builder::basic::kvp("accelerations", accelerations_array));
          }
          
          // time_from_start
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
      
      // multi_dof_joint_trajectoryの変換（もし存在すれば）
      if (!robot_trajectory.multi_dof_joint_trajectory.joint_names.empty()) {
        bsoncxx::builder::basic::document multi_dof_doc;
        
        bsoncxx::builder::basic::array joint_names_array;
        for (const auto& name : robot_trajectory.multi_dof_joint_trajectory.joint_names) {
          joint_names_array.append(name);
        }
        multi_dof_doc.append(bsoncxx::builder::basic::kvp("joint_names", joint_names_array));
        
        // points
        bsoncxx::builder::basic::array points_array;
        for (const auto& point : robot_trajectory.multi_dof_joint_trajectory.points) {
          bsoncxx::builder::basic::document point_doc;
          
          // transforms
          if (!point.transforms.empty()) {
            bsoncxx::builder::basic::array transforms_array;
            for (const auto& transform : point.transforms) {
              bsoncxx::builder::basic::document transform_doc;
              
              // translation
              transform_doc.append(bsoncxx::builder::basic::kvp("translation",
                bsoncxx::builder::basic::make_document(
                  bsoncxx::builder::basic::kvp("x", transform.translation.x),
                  bsoncxx::builder::basic::kvp("y", transform.translation.y),
                  bsoncxx::builder::basic::kvp("z", transform.translation.z)
                )
              ));
              
              // rotation
              transform_doc.append(bsoncxx::builder::basic::kvp("rotation",
                bsoncxx::builder::basic::make_document(
                  bsoncxx::builder::basic::kvp("x", transform.rotation.x),
                  bsoncxx::builder::basic::kvp("y", transform.rotation.y),
                  bsoncxx::builder::basic::kvp("z", transform.rotation.z),
                  bsoncxx::builder::basic::kvp("w", transform.rotation.w)
                )
              ));
              
              transforms_array.append(transform_doc);
            }
            point_doc.append(bsoncxx::builder::basic::kvp("transforms", transforms_array));
          }
          
          // velocities
          if (!point.velocities.empty()) {
            bsoncxx::builder::basic::array velocities_array;
            for (const auto& vel : point.velocities) {
              bsoncxx::builder::basic::document vel_doc;
              
              // linear
              vel_doc.append(bsoncxx::builder::basic::kvp("linear",
                bsoncxx::builder::basic::make_document(
                  bsoncxx::builder::basic::kvp("x", vel.linear.x),
                  bsoncxx::builder::basic::kvp("y", vel.linear.y),
                  bsoncxx::builder::basic::kvp("z", vel.linear.z)
                )
              ));
              
              // angular
              vel_doc.append(bsoncxx::builder::basic::kvp("angular",
                bsoncxx::builder::basic::make_document(
                  bsoncxx::builder::basic::kvp("x", vel.angular.x),
                  bsoncxx::builder::basic::kvp("y", vel.angular.y),
                  bsoncxx::builder::basic::kvp("z", vel.angular.z)
                )
              ));
              
              velocities_array.append(vel_doc);
            }
            point_doc.append(bsoncxx::builder::basic::kvp("velocities", velocities_array));
          }
          
          // accelerations
          if (!point.accelerations.empty()) {
            bsoncxx::builder::basic::array accelerations_array;
            for (const auto& acc : point.accelerations) {
              bsoncxx::builder::basic::document acc_doc;
              
              // linear
              acc_doc.append(bsoncxx::builder::basic::kvp("linear",
                bsoncxx::builder::basic::make_document(
                  bsoncxx::builder::basic::kvp("x", acc.linear.x),
                  bsoncxx::builder::basic::kvp("y", acc.linear.y),
                  bsoncxx::builder::basic::kvp("z", acc.linear.z)
                )
              ));
              
              // angular
              acc_doc.append(bsoncxx::builder::basic::kvp("angular",
                bsoncxx::builder::basic::make_document(
                  bsoncxx::builder::basic::kvp("x", acc.angular.x),
                  bsoncxx::builder::basic::kvp("y", acc.angular.y),
                  bsoncxx::builder::basic::kvp("z", acc.angular.z)
                )
              ));
              
              accelerations_array.append(acc_doc);
            }
            point_doc.append(bsoncxx::builder::basic::kvp("accelerations", accelerations_array));
          }
          
          // time_from_start
          point_doc.append(bsoncxx::builder::basic::kvp("time_from_start", 
            bsoncxx::builder::basic::make_document(
              bsoncxx::builder::basic::kvp("sec", static_cast<int32_t>(point.time_from_start.sec)),
              bsoncxx::builder::basic::kvp("nanosec", static_cast<int32_t>(point.time_from_start.nanosec))
            )
          ));
          
          points_array.append(point_doc);
        }
        multi_dof_doc.append(bsoncxx::builder::basic::kvp("points", points_array));
        
        trajectory_doc.append(bsoncxx::builder::basic::kvp("multi_dof_joint_trajectory", multi_dof_doc));
      }
      
      // インデックスをキーとして追加 ("1", "2", "3"...)
      plan_doc.append(bsoncxx::builder::basic::kvp(std::to_string(index), trajectory_doc));
      index++;
    }
    
    std::string plan_json = bsoncxx::to_json(plan_doc.view());
    
    if(UpdateParamInDBFromJson(used_model_name_, used_record_name_, "plan", plan_json))
    {
      RCLCPP_INFO(this->get_logger(), "Successfully saved %zu plan(s) to database", result.result->plan.size());
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to save plan to database");
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Exception while saving plan: %s", e.what());
  }
  if(CustomUpdateParamInDB(used_model_name_, used_record_name_, "LOCK_FLG", std::vector<bool>{false}))
  {
    RCLCPP_INFO(this->get_logger(), "LOCK_FLG is set to false");
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to set LOCK_FLG to false");
  }
}
/*******************/

int main(int argc, char* argv[])
{
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PrimitiveExcavatorChangePoseFromPose>());
  rclcpp::shutdown();
  return 0;
}
