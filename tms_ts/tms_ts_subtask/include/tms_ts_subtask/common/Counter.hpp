#ifndef COUNTER_NODE_H
#define COUNTER_NODE_H

#include "behaviortree_cpp_v3/action_node.h"
#include <string>
#include <iostream>

namespace BT
{
class Counter : public SyncActionNode
{
public:
  Counter(const std::string& name, const NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
    setRegistrationID("Counter");
  }

  static PortsList providedPorts()
  {
    return { InputPort<std::string>("key") };
  }

private:
  NodeStatus tick() override
  {
    // 1) ポートからキー名を取得
    std::string key;
    if (!getInput("key", key)) {
      throw RuntimeError("missing port [key]");
    }

    // 2) Blackboard から文字列として現在値を取得
    std::string value_str;
    if (!config().blackboard->get(key, value_str)) {
      std::cout << "[Counter] key=\"" << key << "\" not found on blackboard.\n";
      return NodeStatus::FAILURE;
    }

    // 3) 文字列を整数に変換
    int num = 0;
    try {
      num = std::stoi(value_str);
    } catch (const std::exception& e) {
      std::cout << "[Counter] failed to convert '" << value_str << "' to int: " << e.what() << "\n";
      return NodeStatus::FAILURE;
    }

    // 4) インクリメント
    num++;

    // 5) 整数を文字列に戻して Blackboard に保存
    std::string new_value = std::to_string(num);
    config().blackboard->set(key, new_value);

    std::cout << "[Counter] updated key=\"" << key << "\": "
              << value_str << " -> " << new_value << "\n";
    return NodeStatus::SUCCESS;
  }
};
} // namespace BT

#endif // COUNTER_NODE_H
