#ifndef BLACKBOARD_VALUE_WRITER_SRV_NODE_HPP
#define BLACKBOARD_VALUE_WRITER_SRV_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "tms_msg_ts/srv/blackboard_parameters.hpp"

using namespace BT;

class BlackboardValueWriterSrv : public SyncActionNode
{
public:
    BlackboardValueWriterSrv(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config), value_(0)
    { 
        node_ = rclcpp::Node::make_shared("srv_cli_blackboard_node");
        client_ = node_->create_client<tms_msg_ts::srv::BlackboardParameters>("/srv_cli_blackboard");
    }

    ~BlackboardValueWriterSrv()
    {
        rclcpp::shutdown();
    }

    static PortsList providedPorts()
    {
        return { OutputPort<int>("parameter_value"), InputPort<int>("parameter_value") };
    }

    NodeStatus tick() override
    {
        auto request = std::make_shared<tms_msg_ts::srv::BlackboardParameters::Request>();
        Optional<int> port_name = getInput<int>("parameter_value");

        if (!port_name)
        {
            throw RuntimeError("missing required input [port_name]: ", port_name.error());
        }

        request->port_name = port_name.value();

        auto future = client_->async_send_request(request);

        try {
            auto response = future.get();
            value_ = response->val;
            setOutput("parameter_value", value_);
            // RCLCPP_INFO(node_->get_logger(), "Received value: %d at time: %d.%d", response->val, response->time.sec, response->time.nanosec);
            return NodeStatus::SUCCESS;
        } catch (const std::exception &e) {
            // RCLCPP_ERROR(node_->get_logger(), "Service call failed: %s", e.what());
            return NodeStatus::FAILURE;
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<tms_msg_ts::srv::BlackboardParameters>::SharedPtr client_;
    int value_;
};

#endif
