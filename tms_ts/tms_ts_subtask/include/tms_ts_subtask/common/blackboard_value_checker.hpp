#ifndef BLACKBOARD_VALUE_CHECKER_NODE_HPP
#define BLACKBOARD_VALUE_CHECKER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
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
        node_ = rclcpp::Node::make_shared("input");
        pub_ = node_->create_publisher<std_msgs::msg::Int32>("/output_val", 10);
    }

    static PortsList providedPorts()
    {
        return { InputPort<int>("parameter_value") };
    }

    virtual NodeStatus tick() override
    {
        Optional<int> msg = getInput<int>("parameter_value");

        if (!msg)
        {
            throw RuntimeError("missing required input [parameter_value]: ", msg.error());
        }
        auto msg_data = msg.value();
        std::cout << "parameter_valui: " << msg_data << std::endl;

        std_msgs::msg::Int32 input_msg;
        input_msg.data = msg_data;
        pub_->publish(input_msg);

        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
};

#endif