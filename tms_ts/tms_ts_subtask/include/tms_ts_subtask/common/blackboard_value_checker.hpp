#ifndef BLACKBOARD_VALUE_CHECKER_NODE_HPP
#define BLACKBOARD_VALUE_CHECKER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <thread>
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class BlackboardValueChecker : public SyncActionNode
{
public:
    BlackboardValueChecker(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("blackboard_value_checker");
        // pub_ = node_->create_publisher<std_msgs::msg::String>("/output_val", 10);
    }

    static PortsList providedPorts()
    {
        return { InputPort<std::string>("value"), InputPort<std::string>("key") };
    }

    virtual NodeStatus tick() override
    {
        Optional<std::string> key = getInput<std::string>("key");
        Optional<std::string> value = getInput<std::string>("value");

        if (!value || !key)
        {
            std::cout << "missing required input. Please fill key or value parameters." << std::endl;
            return NodeStatus::FAILURE;
        }
        
        std::cout << "key name : " << key.value() << "   ,   value : " << value.value() << std::endl;

        // std_msgs::msg::String input_msg;
        // input_msg.data = msg_data;
        // pub_->publish(input_msg);

        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

#endif