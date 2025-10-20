#ifndef SET_LOCAL_BLACKBOARD_NODE_H
#define SET_LOCAL_BLACKBOARD_NODE_H

#include "behaviortree_cpp_v3/action_node.h"
#include <regex>
#include <sstream>

namespace BT
{
class SetLocalBlackboard : public SyncActionNode
{
public:
  SetLocalBlackboard(const std::string& name, const NodeConfiguration& config) :
    SyncActionNode(name, config)
  {
    setRegistrationID("SetLocalBlackboard");
  }

  static PortsList providedPorts()
  {
    return {InputPort<std::string>("value"), BidirectionalPort("output_key")};
  }

private:
  virtual BT::NodeStatus tick() override
  {
    std::string output_key;
    if (!getInput("output_key", output_key))
    {
      throw RuntimeError("[SetLocalBlackboard] missing port [output_key]");
    }

    std::string value_str;
    if (!getInput("value", value_str))
    {
      throw RuntimeError("[SetLocalBlackboard] missing port [value]");
    }

    // Handle boolean values explicitly
    if (value_str == "true")
    {
      config().blackboard->set(output_key, true); // Set as bool true
    }
    else if (value_str == "false")
    {
      config().blackboard->set(output_key, false); // Set as bool false
    }
    else
    {
      // Check if the value is a number
      std::regex int_regex("^-?\\d+$");
      std::regex double_regex("^-?\\d*\\.\\d+$");

      if (std::regex_match(value_str, int_regex))
      {
        int value_int;
        std::istringstream(value_str) >> value_int;
        config().blackboard->set(output_key, value_int); // Set as int
      }
      else if (std::regex_match(value_str, double_regex))
      {
        double value_double;
        std::istringstream(value_str) >> value_double;
        config().blackboard->set(output_key, value_double); // Set as double
      }
      else
      {
        // Default: Set as string
        config().blackboard->set(output_key, value_str);
      }
    }

    return NodeStatus::SUCCESS;
  }
};
}   // namespace BT

#endif // SET_LOCAL_BLACKBOARD_NODE_H
