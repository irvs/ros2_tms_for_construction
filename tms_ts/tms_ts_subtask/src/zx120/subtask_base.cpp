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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

using std::placeholders::_1;
using namespace BT;
using namespace std::chrono_literals;

class BaseClassZx120Subtasks : public SyncActionNode, public rclcpp::Node{
public:
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "/emergency_signal", 10, std::bind(&BaseClassZx120Subtasks::shutdown_process, this, _1));
    virtual NodeStatus shutdown_process(){
        RCLCPP_INFO_STREAM(node_->get_logger(), "shutdown process is occured !  process name: ");
        return NodeStatus::FAILURE;;    
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Node::SharedPtr node_;
}

// クラスに標準装備する機能
// 1. public SyncActionNodeを継承
// 2. いかなる操作の途中でもユーザ側から処理を強制終了する機能　(GUIボタン(処理強制終了用)の入力をうけて、すべてのノードを強制終了する機能を実装)
// 3. dd
