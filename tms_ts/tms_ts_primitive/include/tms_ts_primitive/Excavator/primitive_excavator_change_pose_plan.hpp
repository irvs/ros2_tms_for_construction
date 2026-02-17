// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
// Licensed under the Apache License, Version 2.0

#ifndef PRIMITIVE_EXCAVATOR_CHANGE_POSE_PLAN_HPP
#define PRIMITIVE_EXCAVATOR_CHANGE_POSE_PLAN_HPP

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

#include "tms_msg_rp/action/tms_rp_excavator.hpp"
#include "tms_msg_rp/msg/tms_rp_excavator_joint_values.hpp"
#include "tms_msg_rp/srv/tms_rp_excavator_param_get.hpp"
#include "tms_msg_rp/srv/tms_rp_excavator_param_set.hpp"

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

class PrimitiveExcavatorChangePosePlan : public PrimitiveNodeBase
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;

  using TmsRpExcavator = tms_msg_rp::action::TmsRpExcavator;
  using GoalHandleTmsRpExcavator = rclcpp_action::ClientGoalHandle<TmsRpExcavator>;

  PrimitiveExcavatorChangePosePlan();

private:
  rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
  rclcpp::Client<tms_msg_rp::srv::TmsRpExcavatorParamGet>::SharedPtr param_get_client_;
  rclcpp::Client<tms_msg_rp::srv::TmsRpExcavatorParamSet>::SharedPtr param_set_client_;
  std::string used_model_name_;
  std::string used_record_name_;
  std::string previous_target_record_name_;
  std::map<std::string, std::string> param_from_db_;
  std::map<std::string, std::string> previous_param_from_db_;
  std::string planning_group_;
  tms_msg_rp::msg::TmsRpExcavatorJointValues current_joint_values;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // Member as an action client
  rclcpp_action::Client<TmsRpExcavator>::SharedPtr action_client_;
  std::shared_future<GoalHandleTmsRpExcavator::SharedPtr> client_future_goal_handle_;
  void goal_response_callback(const GoalHandleTmsRpExcavator::SharedPtr& goal_handle);
  void feedback_callback(GoalHandleTmsRpExcavator::SharedPtr,
                         const std::shared_ptr<const TmsRpExcavator::Feedback> feedback);
  void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                       const GoalHandleTmsRpExcavator::WrappedResult& result);
  
  // Helper methods
  bool parse_previous_plan(TmsRpExcavator::Goal& goal_msg);
  bool parse_constraints(TmsRpExcavator::Goal& goal_msg);
  bool parse_collision_avoidance(TmsRpExcavator::Goal& goal_msg);
  bool parse_planning_scene(TmsRpExcavator::Goal& goal_msg);
  void save_plan_to_db(const TmsRpExcavator::Result::SharedPtr& result);
};

#endif
