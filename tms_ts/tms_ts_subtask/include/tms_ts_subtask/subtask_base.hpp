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
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"

#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/stream/document.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/instance.hpp>


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
    std::map<std::string, int> GetParamFromDB(std::string parts_name);
    // NodeStatus emergency();
    // void StartEmergencyThread();
    rclcpp::Node::SharedPtr node_;
    // mongocxx::instance& db_instance = DatabaseManager::getInstance();
    std::mutex db_instance_mutex;
    // bool emergency_thread_running_ = false;
    // std::thread emergency_thread_;
    // std::string emergency_status_;
};

#endif
