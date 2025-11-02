#include "behaviortree_cpp_v3/bt_factory.h"
#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace BT
{

class LoggerNode : public SyncActionNode
{
public:
  LoggerNode(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
    , node_{rclcpp::Node::make_shared("logger_node")}
  {
    setRegistrationID("LoggerNode");
  }

  static PortsList providedPorts()
  {
    return {
      InputPort<std::string>("text", "ログに出力する文字列")
    };
  }

  NodeStatus tick() override
  {
    std::string text;
    if (!getInput("text", text))
    {
      RCLCPP_WARN(node_->get_logger(), "[LoggerNode] 'text' port not provided");
      return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "[LoggerNode] %s", text.c_str());
    return NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace BT

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<BT::LoggerNode>("LoggerNode");
}