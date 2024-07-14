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
        node_ = rclcpp::Node::make_shared("param_input_node");
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
        return { OutputPort<int>("parameter_value") };
    }

    // tickŠÖ”‚ÌŽÀ‘•
    NodeStatus tick() override
    {
        switch(received_){
        case true:
        {
            setOutput("parameter_value", value_);
            return NodeStatus::SUCCESS;
            break;
        }
        case false:
        {
            setOutput("parameter_value", 10);
            return NodeStatus::SUCCESS;
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