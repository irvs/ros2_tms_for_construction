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

#ifndef PRIMITIVE_EXCAVATOR_CHANGE_POSE_EXECUTE_FROM_PLAN_RETIME_HPP
#define PRIMITIVE_EXCAVATOR_CHANGE_POSE_EXECUTE_FROM_PLAN_RETIME_HPP

#include <memory>
#include <map>
#include <vector>
#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tms_msg_ts/action/leaf_node_base.hpp"
#include "tms_ts_primitive/primitive_node_base.hpp"
#include "tms_msg_rp/action/tms_rp_excavator.hpp"

#include <rclcpp/qos.hpp>   
#include <rmw/qos_profiles.h>

#include "traj_recorder_msgs/action/traj_follow.hpp"

class PrimitiveExcavatorChangePoseExecuteFromPlan : public PrimitiveNodeBase
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;
  using TmsRpExcavator = tms_msg_rp::action::TmsRpExcavator;
  using GoalHandleTmsRpExcavator = rclcpp_action::ClientGoalHandle<TmsRpExcavator>;

  PrimitiveExcavatorChangePoseExecuteFromPlan();

private:
  rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
  std::string used_model_name_;
  std::string used_record_name_;
  std::vector<std::map<std::string, std::string>> params_from_db_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  rclcpp_action::Client<TmsRpExcavator>::SharedPtr action_client_;
  std::shared_future<GoalHandleTmsRpExcavator::SharedPtr> client_future_goal_handle_;
  void goal_response_callback(const GoalHandleTmsRpExcavator::SharedPtr& goal_handle);
  void feedback_callback(GoalHandleTmsRpExcavator::SharedPtr,
                         const std::shared_ptr<const TmsRpExcavator::Feedback> feedback);
  void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                       const GoalHandleTmsRpExcavator::WrappedResult& result);

  rclcpp_action::Client<traj_recorder_msgs::action::TrajFollow>::SharedPtr traj_action_client_;
  rclcpp_action::ClientGoalHandle<traj_recorder_msgs::action::TrajFollow>::SharedPtr traj_goal_handle_;
};

#endif