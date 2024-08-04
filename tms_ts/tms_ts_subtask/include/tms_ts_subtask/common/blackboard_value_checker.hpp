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
        return { InputPort<std::string>("key") };
    }

    virtual NodeStatus tick() override
    {
        Optional<std::string> key = getInput<std::string>("key");

        if (!key)
        {
            std::cout << "[BlackboardValueChecker] missing required input. Please fill key parameter." << std::endl;
            return NodeStatus::FAILURE;
        }
        
        auto any_value = config().blackboard->getAny(key.value());
        if (!any_value)
        {
            std::cout << "[BlackboardValueChecker] No value found for key: " << key.value() << std::endl;
            return NodeStatus::FAILURE;
        }

        try
        {
            if (any_value->type() == typeid(int))
            {
                int value = any_value->cast<int>();
                std::cout << "[BlackboardValueChecker] key name : " << key.value() << "   ,   value : " << value << std::endl;
            }
            else if (any_value->type() == typeid(double))
            {
                double value = any_value->cast<double>();
                std::cout << "[BlackboardValueChecker] key name : " << key.value() << "   ,   value : " << value << std::endl;
            }
            else if (any_value->type() == typeid(bool))
            {
                bool value = any_value->cast<bool>();
                std::cout << "[BlackboardValueChecker] key name : " << key.value() << "   ,   value : " << std::boolalpha << value << std::endl;
            }
            else if (any_value->type() == typeid(std::string))
            {
                std::string value = any_value->cast<std::string>();
                std::cout << "[BlackboardValueChecker] key name : " << key.value() << "   ,   value : " << value << std::endl;
            }
            else
            {
                std::cout << "[BlackboardValueChecker] Unsupported type for key: " << key.value() << std::endl;
                return NodeStatus::FAILURE;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "[BlackboardValueChecker] Error retrieving value for key: " << key.value() << ". " << e.what() << std::endl;
            return NodeStatus::FAILURE;
        }

        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

#endif
