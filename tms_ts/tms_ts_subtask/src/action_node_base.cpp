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

#include "tms_ts_subtask/action_node_base.hpp"

using std::placeholders::_1;
using namespace BT;
using namespace std::chrono_literals;

Zx120ActionClient::Zx120ActionClient() : Node("zx120_action_client") {
    this->client_ptr_ = rclcpp_action::create_client<tms_msg_ts::action::ActionSample>(
      this,
      "zx120_sample");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&Zx120ActionClient::send_goal, this));
}

void Zx120ActionClient::send_goal() {
    this->timer_->cancel();
    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = tms_msg_ts::action::ActionSample::Goal();
    goal_msg.goal_pos = -40;
    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options = rclcpp_action::Client<tms_msg_ts::action::ActionSample>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&Zx120ActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&Zx120ActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&Zx120ActionClient::result_callback, this, std::placeholders::_1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options); 
}

void Zx120ActionClient::goal_response_callback(GoalHandle::SharedPtr future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void Zx120ActionClient::feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const tms_msg_ts::action::ActionSample::Feedback> feedback) {
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing boom position: " << feedback->current_pos << " [deg]");
}

void Zx120ActionClient::result_callback(const GoalHandle::WrappedResult & result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Complete boom modification");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Canceled boom modification");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Error is occured duaring boom modification");
        return;
    }
    rclcpp::shutdown();
}

BaseClassSubtasks::BaseClassSubtasks(const std::string& name, const NodeConfiguration& config) : CoroActionNode(name, config){
    node_ = rclcpp::Node::make_shared(name);
    DatabaseManager::getInstance();
}

NodeStatus BaseClassSubtasks::tick() {
    rclcpp::spin(std::make_shared<Zx120ActionClient>());
    rclcpp::shutdown();
    return NodeStatus::SUCCESS;
};


// データベースから動的パラメータの値をとってくるための関数
std::map<std::string, int> BaseClassSubtasks::GetParamFromDB(int parameter_id){
    //std::lock_guard<std::mutex> lock(db_instance_mutex);
    mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
    mongocxx::database db = client["rostmsdb"];
    mongocxx::collection collection = db["parameter"];
    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "parameter_id" << parameter_id;
    auto filter = filter_builder.view();
    auto result = collection.find_one(filter);
    if (result) {
        std::map<std::string, int> dataMap;

        for (auto&& element : result->view()) {
            std::string key = element.key().to_string();
            if (key != "_id" && key != "type" && key != "description" && key != "parameter_id" && key != "parameter_type" && key != "parts_name") {
                if (element.type() == bsoncxx::type::k_int32) {
                    int value = element.get_int32();
                    dataMap[key] = value;
                }
            }
        }

        return dataMap;
    } else {
        std::cout << "Dynamic parameter not found in your parameter collection" << std::endl;
        return std::map<std::string, int>();
    }
}