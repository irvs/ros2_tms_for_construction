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

#ifndef PRIMITIVE_EXCAVATOR_CHANGE_POSE_PLAN_FROM_POSES_HPP
#define PRIMITIVE_EXCAVATOR_CHANGE_POSE_PLAN_FROM_POSES_HPP

#include <memory>
#include <map>

#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tms_msg_ts/action/leaf_node_base.hpp"
#include "tms_ts_primitive/primitive_node_base.hpp"

#include "tms_msg_rp/action/tms_rp_excavator_change_pose_plan.hpp"

#include <rclcpp/qos.hpp>   
#include <rmw/qos_profiles.h>  

#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/visibility_constraint.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PrimitiveExcavatorChangePoseFromPose : public PrimitiveNodeBase
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;

  using ExcavatorChangePoseFromPose = tms_msg_rp::action::TmsRpExcavatorChangePosePlan;
  using GoalHandleExcavatorChangePoseFromPose = rclcpp_action::ClientGoalHandle<ExcavatorChangePoseFromPose>;

  PrimitiveExcavatorChangePoseFromPose();

private:
  rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
  std::string used_model_name_;
  std::string used_record_name_;
  std::map<std::string, std::string> param_from_db_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // Member as an action client
  rclcpp_action::Client<ExcavatorChangePoseFromPose>::SharedPtr action_client_;
  std::shared_future<GoalHandleExcavatorChangePoseFromPose::SharedPtr> client_future_goal_handle_;
  void goal_response_callback(const GoalHandleExcavatorChangePoseFromPose::SharedPtr& goal_handle);
  void feedback_callback(GoalHandleExcavatorChangePoseFromPose::SharedPtr,
                         const std::shared_ptr<const ExcavatorChangePoseFromPose::Feedback> feedback);
  void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                       const GoalHandleExcavatorChangePoseFromPose::WrappedResult& result);
};

#endif