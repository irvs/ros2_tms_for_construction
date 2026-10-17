#ifndef SET_CURRENT_TIME_TO_LOCAL_BLACKBOARD_H
#define SET_CURRENT_TIME_TO_LOCAL_BLACKBOARD_H

#include "behaviortree_cpp_v3/action_node.h"

#include <chrono>
#include <string>

namespace BT
{

class SetCurrentTimeToLocalBlackboard : public SyncActionNode
{
public:
  SetCurrentTimeToLocalBlackboard(
    const std::string& name,
    const NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
    setRegistrationID("SetCurrentTimeToLocalBlackboard");
  }

  static PortsList providedPorts()
  {
    return {
      BidirectionalPort<std::string>("output_key")
    };
  }

private:
  BT::NodeStatus tick() override
  {
    // Blackboardへの出力先キーを取得
    std::string output_key;

    if (!getInput("output_key", output_key))
    {
      throw RuntimeError(
        "[SetCurrentTimeToLocalBlackboard] missing port [output_key]");
    }

    // 現在時刻を取得（UNIX時間）
    const auto now = std::chrono::system_clock::now();

    const auto duration =
      now.time_since_epoch();

    const double current_time =
      std::chrono::duration<double>(duration).count();

    // Blackboardにdouble型で格納
    config().blackboard->set(
      output_key,
      current_time);

    return NodeStatus::SUCCESS;
  }
};

}  // namespace BT

#endif  // SET_CURRENT_TIME_NODE_H