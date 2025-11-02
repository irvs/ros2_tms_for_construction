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

#ifndef SUBTASK_EXCAVATOR_EXCAVATE_SIMPLE_HPP
#define SUBTASK_EXCAVATOR_EXCAVATE_SIMPLE_HPP

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
#include "tms_ts_subtask/subtask_node_base.hpp"

#include "tms_msg_rp/action/tms_rp_excavator_excavate_simple.hpp"

#include <rclcpp/qos.hpp>   
#include <rmw/qos_profiles.h>  

class SubtaskExcavatorExcavateSimple : public SubtaskNodeBase
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;

  using ExcavatorExcavateSimple = tms_msg_rp::action::TmsRpExcavatorExcavateSimple;
  using GoalHandleExcavatorExcavateSimple = rclcpp_action::ClientGoalHandle<ExcavatorExcavateSimple>;

  SubtaskExcavatorExcavateSimple();

private:
  rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
  std::map<std::string, double> param_from_db_;
  std::string used_model_name_;
  std::string used_record_name_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute(const std::shared_ptr<GoalHandle> goal_handle);

  // Member as an action client
  rclcpp_action::Client<ExcavatorExcavateSimple>::SharedPtr action_client_;
  std::shared_future<GoalHandleExcavatorExcavateSimple::SharedPtr> client_future_goal_handle_;
  void goal_response_callback(const GoalHandleExcavatorExcavateSimple::SharedPtr& goal_handle);
  void feedback_callback(GoalHandleExcavatorExcavateSimple::SharedPtr,
                         const std::shared_ptr<const ExcavatorExcavateSimple::Feedback> feedback);
  void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                       const GoalHandleExcavatorExcavateSimple::WrappedResult& result);
    std::string db_parameter;
};

#endif