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
    return {BidirectionalPort("local_blackboard_key")};
  }

private:
  virtual BT::NodeStatus tick() override
  {
    std::string local_blackboard_key;
    if (!getInput("local_blackboard_key", local_blackboard_key))
    {
      throw RuntimeError("missing port [local_blackboard_key]");
    }

    int num;
    if (!config().blackboard->get(local_blackboard_key, num)) {
      std::cout << "[Counter] Parameter " << local_blackboard_key << "was not found in your local blackboard." << std::endl;
      return NodeStatus::FAILURE;
    }

    num++;
    config().blackboard->set(local_blackboard_key, num);

    std::cout << "[Counter] Parameter " << local_blackboard_key << "was set " << std::to_string(num) << "." << std::endl;

    return NodeStatus::SUCCESS;
  }
};
}   // namespace BT

#endif // COUNTER_NODE_H
