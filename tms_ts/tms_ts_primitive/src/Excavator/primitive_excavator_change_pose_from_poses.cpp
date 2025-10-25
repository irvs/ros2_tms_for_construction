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

#include "tms_ts_primitive/Excavator/primitive_excavator_change_pose_from_poses.hpp"
#include <glog/logging.h>

using namespace std::chrono_literals;

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

  action_client_ = rclcpp_action::create_client<ExcavatorChangePoseFromPose>(this, "tms_rp_excavator_change_pose_from_poses",nullptr ,options_client);
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
  param_from_db_ = GetParamFromDBAsJson(goal->model_name, goal->record_name);
  if (param_from_db_.empty())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to get parameters from DB");
    return rclcpp_action::GoalResponse::REJECT;
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
  RCLCPP_INFO(this->get_logger(), "Get pose from DB.");

  RCLCPP_INFO(this->get_logger(), "param_from_db_ contents:");
  for (const auto& [key, value] : param_from_db_)
  {
    RCLCPP_INFO(this->get_logger(), "  %s: %s", key.c_str(), value.c_str());
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
    size_t array_size = std::distance(position_array.begin(), position_array.end());
    
    // 配列サイズのチェック（2個以上必要）
    if (array_size < 2) {
      RCLCPP_ERROR(this->get_logger(), "position_with_angle array must contain at least 2 elements, but got %zu", array_size);
      handle_error("position_with_angle array must contain at least 2 elements");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Found %zu positions in position_with_angle array", array_size);
    
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
      
      target_pose.position.x = pos_doc["x"].get_double().value;
      target_pose.position.y = pos_doc["y"].get_double().value;
      target_pose.position.z = pos_doc["z"].get_double().value;
      target_pose.theta_w = pos_doc["theta_w"].get_double().value;
      
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
            if (jc_doc["position"]) joint_constraint.position = jc_doc["position"].get_double().value;
            if (jc_doc["tolerance_above"]) joint_constraint.tolerance_above = jc_doc["tolerance_above"].get_double().value;
            if (jc_doc["tolerance_below"]) joint_constraint.tolerance_below = jc_doc["tolerance_below"].get_double().value;
            if (jc_doc["weight"]) joint_constraint.tolerance_below = jc_doc["weight"].get_double().value;
            
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
              if (offset["x"]) position_constraint.target_point_offset.x = offset["x"].get_double().value;
              if (offset["y"]) position_constraint.target_point_offset.y = offset["y"].get_double().value;
              if (offset["z"]) position_constraint.target_point_offset.z = offset["z"].get_double().value;
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
                      solid_primitive.dimensions.push_back(dim.get_double().value);
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
                    if (pos["x"]) geo_pose.position.x = pos["x"].get_double().value;
                    if (pos["y"]) geo_pose.position.y = pos["y"].get_double().value;
                    if (pos["z"]) geo_pose.position.z = pos["z"].get_double().value;
                  }
                  
                  if (pose_doc["orientation"]) {
                    auto ori = pose_doc["orientation"].get_document().value;
                    if (ori["x"]) geo_pose.orientation.x = ori["x"].get_double().value;
                    if (ori["y"]) geo_pose.orientation.y = ori["y"].get_double().value;
                    if (ori["z"]) geo_pose.orientation.z = ori["z"].get_double().value;
                    if (ori["w"]) geo_pose.orientation.w = ori["w"].get_double().value;
                  }
                  
                  position_constraint.constraint_region.primitive_poses.push_back(geo_pose);
                }
              }
            }
            
            // weight
            if (pc_doc["weight"]) {
              if (pc_doc["weight"].type() == bsoncxx::type::k_double) {
                position_constraint.weight = pc_doc["weight"].get_double().value;
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
              if (ori["x"]) orientation_constraint.orientation.x = ori["x"].get_double().value;
              if (ori["y"]) orientation_constraint.orientation.y = ori["y"].get_double().value;
              if (ori["z"]) orientation_constraint.orientation.z = ori["z"].get_double().value;
              if (ori["w"]) orientation_constraint.orientation.w = ori["w"].get_double().value;
            }
            
            // tolerances
            if (oc_doc["absolute_x_axis_tolerance"]) orientation_constraint.absolute_x_axis_tolerance = oc_doc["absolute_x_axis_tolerance"].get_double().value;
            if (oc_doc["absolute_y_axis_tolerance"]) orientation_constraint.absolute_y_axis_tolerance = oc_doc["absolute_y_axis_tolerance"].get_double().value;
            if (oc_doc["absolute_z_axis_tolerance"]) orientation_constraint.absolute_z_axis_tolerance = oc_doc["absolute_z_axis_tolerance"].get_double().value;
            
            // weight
            if (oc_doc["weight"]) {
              if (oc_doc["weight"].type() == bsoncxx::type::k_double) {
                orientation_constraint.weight = oc_doc["weight"].get_double().value;
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
            if (vc_doc["target_radius"]) visibility_constraint.target_radius = vc_doc["target_radius"].get_double().value;
            
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
                  if (pos["x"]) visibility_constraint.target_pose.pose.position.x = pos["x"].get_double().value;
                  if (pos["y"]) visibility_constraint.target_pose.pose.position.y = pos["y"].get_double().value;
                  if (pos["z"]) visibility_constraint.target_pose.pose.position.z = pos["z"].get_double().value;
                }
                if (pose["orientation"]) {
                  auto ori = pose["orientation"].get_document().value;
                  if (ori["x"]) visibility_constraint.target_pose.pose.orientation.x = ori["x"].get_double().value;
                  if (ori["y"]) visibility_constraint.target_pose.pose.orientation.y = ori["y"].get_double().value;
                  if (ori["z"]) visibility_constraint.target_pose.pose.orientation.z = ori["z"].get_double().value;
                  if (ori["w"]) visibility_constraint.target_pose.pose.orientation.w = ori["w"].get_double().value;
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
                  if (pos["x"]) visibility_constraint.sensor_pose.pose.position.x = pos["x"].get_double().value;
                  if (pos["y"]) visibility_constraint.sensor_pose.pose.position.y = pos["y"].get_double().value;
                  if (pos["z"]) visibility_constraint.sensor_pose.pose.position.z = pos["z"].get_double().value;
                }
                if (pose["orientation"]) {
                  auto ori = pose["orientation"].get_document().value;
                  if (ori["x"]) visibility_constraint.sensor_pose.pose.orientation.x = ori["x"].get_double().value;
                  if (ori["y"]) visibility_constraint.sensor_pose.pose.orientation.y = ori["y"].get_double().value;
                  if (ori["z"]) visibility_constraint.sensor_pose.pose.orientation.z = ori["z"].get_double().value;
                  if (ori["w"]) visibility_constraint.sensor_pose.pose.orientation.w = ori["w"].get_double().value;
                }
              }
            }
            
            // angles
            if (vc_doc["max_view_angle"]) visibility_constraint.max_view_angle = vc_doc["max_view_angle"].get_double().value;
            if (vc_doc["max_range_angle"]) visibility_constraint.max_range_angle = vc_doc["max_range_angle"].get_double().value;
            
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
                visibility_constraint.weight = vc_doc["weight"].get_double().value;
              }
            }
            
            goal_msg.constraints.visibility_constraints.push_back(visibility_constraint);
          }
          RCLCPP_INFO(this->get_logger(), "Added %zu visibility constraints", goal_msg.constraints.visibility_constraints.size());
        }
      }
    }

    // TODO: Planning Sceneの設定（必要に応じて追加）

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
