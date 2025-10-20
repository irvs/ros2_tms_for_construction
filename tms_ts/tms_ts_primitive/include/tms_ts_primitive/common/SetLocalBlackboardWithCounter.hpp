#ifndef SET_LOCAL_BLACKBOARD_WITH_COUNTER_NODE_H
#define SET_LOCAL_BLACKBOARD_WITH_COUNTER_NODE_H

#include "behaviortree_cpp_v3/action_node.h"
#include <regex>
#include <sstream>

namespace BT
{
class SetLocalBlackboardWithCounter : public SyncActionNode
{
public:
  SetLocalBlackboardWithCounter(const std::string& name, const NodeConfiguration& config) :
    SyncActionNode(name, config)
  {
    setRegistrationID("SetLocalBlackboardWithCounter");
  }

  static PortsList providedPorts()
  {
    return {InputPort<std::string>("key"), BidirectionalPort<std::string>("counter_key")};
  }

private:
  virtual BT::NodeStatus tick() override
  {
    std::string key;
    if (!getInput("key", key))
    {
      throw RuntimeError("[SetLocalBlackboardWithCounter] missing port [key]");
    }

    std::string counter_key;
    if (!getInput("counter_key", counter_key))
    {
      throw RuntimeError("[SetLocalBlackboardWithCounter] missing port [counter_key]");
    }

    std::string record_name;
    if (!config().blackboard->get(key, record_name)) {
      std::cout << "[SetLocalBlackboardWithCounter] Parameter " << counter_key << "was not found in your local blackboard." << std::endl;
      return NodeStatus::FAILURE;
    }

    std::string num;
    if (!config().blackboard->get(counter_key, num)) {
      std::cout << "[SetLocalBlackboardWithCounter] Parameter " << counter_key << "was not found in your local blackboard." << std::endl;
      return NodeStatus::FAILURE;
    }

    std::string stripped = std::regex_replace(record_name, std::regex("\\d+$"), "");
    record_name = stripped + num;

    std::cout << "[SetLocalBlackboardWithCounter] Parameter " << key << "was set " << record_name << "." << std::endl;

    config().blackboard->set(key, record_name);

    return NodeStatus::SUCCESS;
  }
};
}   // namespace BT

#endif // SET_LOCAL_BLACKBOARD_WITH_COUNTER_NODE_H
