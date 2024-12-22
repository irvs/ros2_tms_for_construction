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

#ifndef LEAF_NODE_BASE_HPP
#define LEAF_NODE_BASE_HPP

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

#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>

#include "tms_msg_ts/action/leaf_node_base.hpp"


using std::placeholders::_1;
using namespace BT;
using namespace std::chrono_literals;



class LeafNodeBase : public CoroActionNode{
public:
    LeafNodeBase(const std::string& name, const NodeConfiguration& config);
    NodeStatus tick() override;
    std::mutex db_instance_mutex;
    bool should_send_goal_ = true;
    bool goal_result_available_ = false;
    bool goal_updated_ = false;
    int cancel_process_count_ = 5;
    
    std::string subtask_name_;

    void createActionClient(const std::string & action_name_); 

    void send_new_goal();
    void halt_bef();
    void halt() override;
    // on_wait_for_result()はserverから返ってきたfeedbackを�?��?する関数。長くブロ�?キングが生じる処�?は実�?しな�?こと
    bool should_cancel_goal();
    bool is_future_goal_handle_complete(std::chrono::milliseconds & elapsed);

    NodeStatus bt_status;
    rclcpp::Time time_goal_sent_;
    std::chrono::milliseconds bt_loop_duration_ = std::chrono::milliseconds(100);
    std::chrono::milliseconds server_timeout_ = std::chrono::milliseconds(1000); // Please specify the timeout duration

    std::shared_ptr<rclcpp_action::Client<tms_msg_ts::action::LeafNodeBase>> action_client_;
    typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::LeafNodeBase>::SharedPtr goal_handle_;
    typename tms_msg_ts::action::LeafNodeBase::Goal goal_;
    typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::LeafNodeBase>::WrappedResult result_;
    std::shared_ptr<const typename tms_msg_ts::action::LeafNodeBase::Feedback> feedback_;
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    std::shared_ptr<std::shared_future<typename rclcpp_action::ClientGoalHandle<tms_msg_ts::action::LeafNodeBase>::SharedPtr>> future_goal_handle_;
};

#endif