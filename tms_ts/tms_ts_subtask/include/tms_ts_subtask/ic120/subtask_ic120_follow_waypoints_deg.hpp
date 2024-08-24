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

#ifndef TMS_TS_SUBTASK_IC120_FOLLOW_WAYPOINTS_DEG_HPP
#define TMS_TS_SUBTASK_IC120_FOLLOW_WAYPOINTS_DEG_HPP

#include <memory>
#include <map>
#include <vector>
#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <iostream>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tms_msg_ts/action/leaf_node_base.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "tms_ts_subtask/subtask_node_base.hpp"

class SubtaskIc120FollowWaypointysDeg : public SubtaskNodeBase
{
public:
    using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
    using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;
    using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

    SubtaskIc120FollowWaypointysDeg();

private:
    rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
    rclcpp_action::Client<FollowWaypoints>::SharedPtr action_client_;
    std::shared_future<GoalHandleFollowWaypoints::SharedPtr> client_future_goal_handle_;
    std::map<std::pair<std::string, std::string>, double> parameters;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);
    void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleFollowWaypoints::SharedPtr,
                           const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                         const GoalHandleFollowWaypoints::WrappedResult& result);
};

#endif  // TMS_TS_SUBTASK_IC120_FOLLOW_WAYPOINTS_HPP
