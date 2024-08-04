#ifndef BLACKBOARD_VALUE_WRITER_SRV_NODE_HPP
#define BLACKBOARD_VALUE_WRITER_SRV_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "tms_msg_ts/srv/blackboard_parameters.hpp"
#include <string>

using namespace BT;

class BlackboardValueWriterSrv : public SyncActionNode
{
public:
    BlackboardValueWriterSrv(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config), value_(0)
    { 
        node_ = rclcpp::Node::make_shared("srv_cli_blackboard_node");
        client_ = node_->create_client<tms_msg_ts::srv::BlackboardParameters>("blackboard_parameters");
    }

    ~BlackboardValueWriterSrv()
    {
        rclcpp::shutdown();
    }

    static PortsList providedPorts()
    {
        return { OutputPort<std::string>("value"), InputPort<std::string>("output_key") };
    }

    NodeStatus tick() override
    {
        Optional<std::string> key_name = getInput<std::string>("output_key");

        if (!key_name)
        {
            std::cout << "[BlackboardValueWriterSrv] Missing required key. Please fill blackboard key name in key_name parameter." << std::endl;
            return NodeStatus::FAILURE;
        }

        while (!client_->wait_for_service(std::chrono::seconds(10))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "client interrupted while waiting for service to appear.");
                return NodeStatus::FAILURE;
                }
            std::cout << "[BlackboardValueWriterSrv] waiting for service to appear..." << std::endl;
            return NodeStatus::RUNNING;
        }
        auto request = std::make_shared<tms_msg_ts::srv::BlackboardParameters::Request>();
        request->port_name = std::string(key_name.value());
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "service call failed ");
            client_->remove_pending_request(result_future);
            return NodeStatus::FAILURE;
        }

        auto result = result_future.get();
        setOutput("value", std::to_string(result->val));

        return NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<tms_msg_ts::srv::BlackboardParameters>::SharedPtr client_;
    int value_;
};

#endif
