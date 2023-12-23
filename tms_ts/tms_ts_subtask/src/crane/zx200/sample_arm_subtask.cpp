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

#include "tms_ts_subtask/crane/zx200/sample_arm_subtask.hpp"
#include <glog/logging.h>

using namespace std::chrono_literals;

SubtaskSampleZx200Arm::SubtaskSampleZx200Arm() : SubtaskNodeBase("subtask_sample_zx200_arm")
{
  publisher_ = this->create_publisher<std_msgs::msg::Float64>("/zx200/arm/cmd", 10);
  this->action_server_ = rclcpp_action::create_server<tms_msg_ts::action::LeafNodeBase>(
      this,
      "sample_zx200_arm_subtask",
      std::bind(&SubtaskSampleZx200Arm::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SubtaskSampleZx200Arm::handle_cancel, this, std::placeholders::_1),
      std::bind(&SubtaskSampleZx200Arm::handle_accepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse SubtaskSampleZx200Arm::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const tms_msg_ts::action::LeafNodeBase::Goal> goal)
{
    parameters = GetParamFromDB(goal->model_name, goal->component_name);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SubtaskSampleZx200Arm::handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel subtask node");
    sleep(5);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void SubtaskSampleZx200Arm::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    using namespace std::placeholders;
    std::thread{std::bind(&SubtaskSampleZx200Arm::execute, this, _1), goal_handle}.detach();
}

void SubtaskSampleZx200Arm::execute(const std::shared_ptr<GoalHandle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "subtask is executing...");
    rclcpp::Rate loop_rate(1);
    const auto goal_pos = parameters["zx200_arm_goal_pos"];
    auto feedback = std::make_shared<tms_msg_ts::action::LeafNodeBase::Feedback>();
    auto & current_pos = feedback->current_pos;
    auto result = std::make_shared<tms_msg_ts::action::LeafNodeBase::Result>();
    int deg = 0;
    std_msgs::msg::Float64 msg_rad;

    while(deg >= goal_pos){
        if (goal_handle->is_canceling()) {
            result->result = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "subtask execution is canceled");
            return;
        }
        deg -= goal_pos / float(20.0);
        msg_rad.data = float(deg * float(M_PI / 180));
        current_pos = deg;
        goal_handle->publish_feedback(feedback); 
        publisher_->publish(msg_rad);
        RCLCPP_INFO_STREAM(this->get_logger(), "Publishing arm position: " << deg << " [deg]");
        sleep(1); 
    }

    if(rclcpp::ok()){
        result->result = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "subtask execution is succeeded");
    }
}

int main(int argc, char *argv[])
{
	// Initialize Google's logging library.
	google::InitGoogleLogging(argv[0]);
	google::InstallFailureSignalHandler();
  
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubtaskSampleZx200Arm>());
    rclcpp::shutdown();
    return 0;
}