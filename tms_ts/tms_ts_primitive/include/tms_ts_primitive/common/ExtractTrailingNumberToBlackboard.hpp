#ifndef EXTRACT_TRAILING_NUMBER_TO_BLACKBOARD_NODE_H
#define EXTRACT_TRAILING_NUMBER_TO_BLACKBOARD_NODE_H

#include "behaviortree_cpp_v3/action_node.h"
#include <regex>
#include <iostream>

namespace BT
{

class ExtractTrailingNumberToBlackboard : public SyncActionNode
{
public:
  ExtractTrailingNumberToBlackboard(
      const std::string& name,
      const NodeConfiguration& config)
    : SyncActionNode(name, config)
  {
    setRegistrationID("ExtractTrailingNumberToBlackboard");
  }

  static PortsList providedPorts()
  {
    return {
      InputPort<std::string>("source_key"),
      InputPort<std::string>("stored_key")
    };
  }

private:
  BT::NodeStatus tick() override
  {
    // 1. 取得元のキー名を取得
    std::string source_key;
    if (!getInput("source_key", source_key))
    {
      throw RuntimeError(
          "[ExtractTrailingNumberToBlackboard] "
          "missing port [source_key]");
    }

    // 2. 保存先のキー名を取得
    std::string stored_key;
    if (!getInput("stored_key", stored_key))
    {
      throw RuntimeError(
          "[ExtractTrailingNumberToBlackboard] "
          "missing port [stored_key]");
    }

    // 3. 取得元の文字列をBlackboardから読み込む
    std::string source_value;
    if (!config().blackboard->get(source_key, source_value))
    {
      std::cout
          << "[ExtractTrailingNumberToBlackboard] "
          << "Source key not found: "
          << source_key << std::endl;

      return NodeStatus::FAILURE;
    }

    // 4. 末尾の数字を抽出する
    std::smatch match;
    std::regex pattern("(\\d+)$");

    if (!std::regex_search(source_value, match, pattern))
    {
      std::cout
          << "[ExtractTrailingNumberToBlackboard] "
          << "No trailing number found in: "
          << source_value << std::endl;

      return NodeStatus::FAILURE;
    }

    // 5. 抽出した数字を取得
    std::string number = match[1].str();

    // 6. 指定されたキーに数字を保存
    config().blackboard->set(stored_key, number);

    std::cout
        << "[ExtractTrailingNumberToBlackboard] "
        << "Extracted " << number
        << " from " << source_value
        << " and saved to " << stored_key
        << std::endl;

    return NodeStatus::SUCCESS;
  }
};

} // namespace BT

#endif // EXTRACT_TRAILING_NUMBER_TO_BLACKBOARD_NODE_H