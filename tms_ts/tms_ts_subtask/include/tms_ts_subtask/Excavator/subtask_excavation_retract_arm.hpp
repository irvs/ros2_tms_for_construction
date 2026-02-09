// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.

#ifndef SUBTASK_EXCAVATION_RETRACT_ARM_HPP_
#define SUBTASK_EXCAVATION_RETRACT_ARM_HPP_

#include <memory>
#include <map>
#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tms_msg_ts/action/leaf_node_base.hpp"
#include "tms_msg_rp/action/tms_rp_excavator.hpp"
#include "tms_msg_rp/msg/tms_rp_excavator_joint_values.hpp"
#include "tms_msg_rp/srv/tms_rp_excavator_param_get.hpp"
#include "tms_msg_rp/srv/tms_rp_excavator_param_set.hpp"
#include "tms_ts_subtask/subtask_node_base.hpp"
#include "tms_ts_subtask/Excavator/lib/excavator_pose_converter.hpp"
#include <geometry_msgs/msg/pose.hpp>

class SubtaskExcavationRetractArm : public SubtaskNodeBase
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;
  using ExcavatorAction = tms_msg_rp::action::TmsRpExcavator;
  using GoalHandleExcavator = rclcpp_action::ClientGoalHandle<ExcavatorAction>;

  SubtaskExcavationRetractArm();

private:
  rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
  rclcpp_action::Client<ExcavatorAction>::SharedPtr action_client_;
  rclcpp::Client<tms_msg_rp::srv::TmsRpExcavatorParamGet>::SharedPtr param_get_client_;
  rclcpp::Client<tms_msg_rp::srv::TmsRpExcavatorParamSet>::SharedPtr param_set_client_;
  
  std::string planning_group_;
  double arm_joint_max_limit_;
  double search_precision_;
  ExcavatorPoseConverter pose_converter_;
  
  std::shared_future<GoalHandleExcavator::SharedPtr> client_future_goal_handle_;
  std::map<std::string, std::string> param_from_db_;
  std::string used_model_name_;
  std::string used_record_name_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);
  
  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle> goal_handle);
  
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // Action client helper - synchronous call
  bool call_excavator_action_sync(
      const ExcavatorAction::Goal& goal,
      ExcavatorAction::Result::SharedPtr& result);

  // Binary search for maximum arm_joint angle
  bool binary_search_max_arm_angle(
      const tms_msg_rp::msg::TmsRpExcavatorJointValues& original_joint_values,
      double original_arm_angle,
      double& best_arm_angle,
      tms_msg_rp::msg::TmsRpExcavatorJointValues& best_joint_values);
};

#endif
