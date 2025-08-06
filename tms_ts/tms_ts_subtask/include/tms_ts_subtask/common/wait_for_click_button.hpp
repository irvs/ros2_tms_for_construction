#ifndef WAIT_FOR_CLICK_NODE_H
#define WAIT_FOR_CLICK_NODE_H

#include <behaviortree_cpp_v3/action_node.h>
#include <atomic>

namespace BT
{

class WaitForClick : public AsyncActionNode
{
public:
  WaitForClick(const std::string& name, const NodeConfiguration& config);
  static PortsList providedPorts();

  // AsyncActionNode ではこの tick() が別スレッドで実行され、
  // 終了までノードステータスは RUNNING のままになります。
  NodeStatus tick() override;
  void halt() override;

private:
  std::atomic<bool> clicked_;
};

} // namespace BT

#endif // WAIT_FOR_CLICK_NODE_H
