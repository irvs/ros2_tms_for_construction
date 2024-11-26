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

#ifndef BLACKBOARD_VALUE_WRITER_TOPIC_NODE_HPP
#define BLACKBOARD_VALUE_WRITER_TOPIC_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <thread>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class BlackboardValueWriterTopic : public SyncActionNode
{
public:
    BlackboardValueWriterTopic(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config), value_(0), received_(false)
    { 
        node_ = rclcpp::Node::make_shared("blackboard_param_writer_topic");
        sub_ = node_->create_subscription<std_msgs::msg::Int32>(
            "/input_val", 10, std::bind(&BlackboardValueWriterTopic::callback, this, std::placeholders::_1));
        spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });
    }

    ~BlackboardValueWriterTopic()
    {
        rclcpp::shutdown();
        if (spin_thread_.joinable()) {
            spin_thread_.join();
        }
    }

    static PortsList providedPorts()
    {
        return { OutputPort<int>("value") , InputPort<std::string>("key")};
    }

    // tickä÷êîÇÃé¿ëï
    NodeStatus tick() override
    {
        switch(received_){
        case true:
        {
            Optional<std::string> key = getInput<std::string>("key");
            Optional<int> value = getInput<int>("value");

            if (!value || !key)
            {
                std::cout << "[BlackboardValueWriterTopic] Missing required input. Please fill key or value parameters." << std::endl;
                return NodeStatus::FAILURE;
            }

            std::cout << "[BlackboardValueWriterTopic] Stored blackboard parameter [" << key.value() << "] : " << int(value_) << std::endl;

            setOutput("value", value_);
            return NodeStatus::SUCCESS;
            break;
        }
        case false:
        {
            std::cout << "[BlackboardValueWriterTopic]  Input value is not subscribed. Please confirm ros2 topic /input_val is publishing nowe" << std::endl;
            return NodeStatus::FAILURE;
            break;
        }
        }
    }

private:
    void callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        value_ = msg->data;
        received_ = true;
    }
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
    int value_;
    bool received_;
    std::thread spin_thread_; 
};

#endif