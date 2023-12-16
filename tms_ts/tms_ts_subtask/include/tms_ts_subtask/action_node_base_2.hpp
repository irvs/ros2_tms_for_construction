// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BASE_CLASS_SUBTASKS_HPP
#define BASE_CLASS_SUBTASKS_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <thread>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

#include "tms_msg_ts/action/action_sample.hpp"


using std::placeholders::_1;
using namespace BT;
using namespace std::chrono_literals;

class DatabaseManager {
public:
    static mongocxx::instance& getInstance() {
        static mongocxx::instance instance{};
        return instance;
    }
};


class BaseClassSubtasks : public CoroActionNode{
public:
    BaseClassSubtasks(const std::string& name, const NodeConfiguration& config);
    NodeStatus tick() override;
    std::map<std::string, int> GetParamFromDB(int parameter_id);
    std::map<std::string, int> GetParamFromDB(std::string parts_name);
    std::mutex db_instance_mutex;
    bool should_send_goal_ = true;
    bool goal_result_available_ = false;
    bool goal_updated_ = false;
    std::string action_name_ = "template_action_node_name"; // Please specify the action node name per corresponding subtasks

    void createActionClient(const std::string & action_name); 
    void send_new_goal();
    void halt() override;

    virtual void on_tick();
    virtual void on_wait_for_result(std::shared_ptr<const typename tms_msg_ts::action::ActionSample::Feedback>){}; //feedbackを受け取ったときの処理はこの関数に実装
    bool is_future_goal_handle_complete(std::chrono::milliseconds & elapsed);

    NodeStatus bt_status;
    rclcpp::Time time_goal_sent_;
    std::chrono::milliseconds bt_loop_duration_;
    std::chrono::milliseconds server_timeout_ = std::chrono::milliseconds(1000); // Please specify the timeout duration

    std::shared_ptr<rclcpp_action::Client<tms_msg_ts::action::ActionSample>> action_client_;
    typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::ActionSample>::SharedPtr goal_handle_;
    typename tms_msg_ts::action::ActionSample::Goal goal_;
    typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::ActionSample>::WrappedResult result_;
    std::shared_ptr<const typename tms_msg_ts::action::ActionSample::Feedback> feedback_;

    std::string action_client_node_name = "action_client_node(Please specify unique action client node name.)";

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::ActionSample>::SharedPtr>> future_goal_handle_;
};

#endif