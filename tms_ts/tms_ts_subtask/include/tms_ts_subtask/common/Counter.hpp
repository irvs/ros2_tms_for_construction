#ifndef COUNTER_NODE_H
#define COUNTER_NODE_H

#include "behaviortree_cpp_v3/action_node.h"
#include <regex>
#include <sstream>

namespace BT
{
class Counter : public SyncActionNode
{
public:
  Counter(const std::string& name, const NodeConfiguration& config) :
    SyncActionNode(name, config)
  {
    setRegistrationID("Counter");
  }

  static PortsList providedPorts()
  {
    return {BidirectionalPort("key")};
  }

private:
  virtual BT::NodeStatus tick() override
  {
    std::string key;
    if (!getInput("key", key))
    {
      throw RuntimeError("missing port [key]");
    }

    int num;
    if (!config().blackboard->get(key, num)) {
      std::cout << "[Counter] Parameter " << key << "was not found in your local blackboard." << std::endl;
      return NodeStatus::FAILURE;
    }

    num++;
    config().blackboard->set(key, num);

    std::cout << "[Counter] Parameter " << key << "was set " << std::to_string(num) << "." << std::endl;

    return NodeStatus::SUCCESS;
  }
};
}   // namespace BT

#endif // COUNTER_NODE_H
