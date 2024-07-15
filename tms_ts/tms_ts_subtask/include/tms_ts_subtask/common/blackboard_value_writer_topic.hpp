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
                std::cout << "missing required input. Please fill key or value parameters." << std::endl;
                return NodeStatus::FAILURE;
            }

            std::cout << "Stored blackboard parameter [" << key.value() << "] : " << int(value_) << std::endl;

            setOutput("value", value_);
            return NodeStatus::SUCCESS;
            break;
        }
        case false:
        {
            std::cout << "input value is not subscribed. Please confirm ros2 topic /input_val is publishing nowe" << std::endl;
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