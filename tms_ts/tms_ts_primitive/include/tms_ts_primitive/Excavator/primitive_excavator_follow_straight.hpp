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

#ifndef TMS_TS_PRIMITIVE_EXCAVATOR_FOLLOW_STRAIGHT_HPP
#define TMS_TS_PRIMITIVE_EXCAVATOR_FOLLOW_STRAIGHT_HPP

#include <memory>
#include <map>
#include <vector>
#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav_msgs/msg/path.hpp"
#include "nav2_msgs/action/follow_path.hpp"
#include "tms_msg_rp/action/tms_rp_follow_straight.hpp"

#include "tms_msg_ts/action/leaf_node_base.hpp"
#include "tms_ts_primitive/primitive_node_base.hpp"



class PrimitiveExcavatorFollowStraight : public PrimitiveNodeBase
{
public:
    using FollowStraight = tms_msg_rp::action::TmsRpFollowStraight;
    using GoalHandle = rclcpp_action::ServerGoalHandle<tms_msg_ts::action::LeafNodeBase>;
    using GoalHandleFollowStraight = rclcpp_action::ClientGoalHandle<FollowStraight>;

    PrimitiveExcavatorFollowStraight();

private:
    rclcpp_action::Server<tms_msg_ts::action::LeafNodeBase>::SharedPtr action_server_;
    rclcpp_action::Client<FollowStraight>::SharedPtr action_client_;
    std::shared_future<GoalHandleFollowStraight::SharedPtr> client_future_goal_handle_;
    std::map<std::string, double> parameters;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
    void execute(const std::shared_ptr<GoalHandle> goal_handle);
    void goal_response_callback(const GoalHandleFollowStraight::SharedPtr& goal_handle);
    void feedback_callback(GoalHandleFollowStraight::SharedPtr,
                           const std::shared_ptr<const FollowStraight::Feedback> feedback);
    void result_callback(const std::shared_ptr<GoalHandle> goal_handle,
                         const GoalHandleFollowStraight::WrappedResult& result);
};

#endif  // TMS_TS_PRIMITIVE_EXCAVATOR_FOLLOW_STRAIGHT_HPP
