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
#include <iostream>

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

#include "tms_ts_subtask/subtask_base.hpp"

BaseClassSubtasks::BaseClassSubtasks(const std::string& name, const NodeConfiguration& config) : CoroActionNode(name, config){
    node_ = rclcpp::Node::make_shared(name);
    // subscription_ = node_->create_subscription<std_msgs::msg::String>(
    //     "/emergency_signal", 10, std::bind(&BaseClassSubtasks::shutdown_node, this, _1));
}

int BaseClassSubtasks::GetParamFromDB(std::string parts_name, std::string param_name){
    mongocxx::instance instance{};
    mongocxx::client client{mongocxx::uri{"mongodb://localhost:27017"}};
    mongocxx::database db = client["rostmsdb"];
    mongocxx::collection collection = db["parameter"];
    bsoncxx::builder::stream::document filter_builder;
    filter_builder << "parts_name" << parts_name;
    auto filter = filter_builder.view();
    auto result = collection.find_one(filter);
    if (result) {
        int value = static_cast<float>(result->view()[param_name].get_int32()); 
        return value;
    } else {
        std::cerr << "Data not found" << std::endl;
        return 0;
    }
}

// NodeStatus BaseClassSubtasks::shutdown_node(const std_msgs::msg::String & msg) const {
//     RCLCPP_INFO_STREAM(node_->get_logger(), "shutdown process is occured !");
//     return NodeStatus::FAILURE;
// }

