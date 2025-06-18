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

#ifndef SAMPLE_SUBTASK_MST110CR_NAVIGATE_ANYWHERE_DEG_HPP
#define SAMPLE_SUBTASK_MST110CR_NAVIGATE_ANYWHERE_DEG_HPP

#include <memory>
#include <map>

#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <sstream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"

#include "tms_msg_ts/action/leaf_node_base.hpp"
#include "tms_ts_subtask/subtask_node_base.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"


class SubtaskMst110crNavigateAnywhereDeg : public SubtaskNodeBase
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleMst110crNavigateAnywhere = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    SubtaskMst110crNavigateAnywhereDeg();

private:
    rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);

    // Member as an action client
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
    std::shared_future<GoalHandleMst110crNavigateAnywhere::SharedPtr> client_future_goal_handle_;
    std::map<std::string, double> parameters;
    void goal_response_callback(const GoalHandleMst110crNavigateAnywhere::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleMst110crNavigateAnywhere::SharedPtr,
                            const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                        const GoalHandleMst110crNavigateAnywhere::WrappedResult& result);
};

#endif