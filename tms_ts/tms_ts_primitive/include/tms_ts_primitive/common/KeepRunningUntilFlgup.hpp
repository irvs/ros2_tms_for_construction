// Copyright 2023, IRVS Laboratory, Kyushu University, Japan.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
// http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef KEEP_RUNNING_UNTIL_FLGUP_NODE_HPP
#define KEEP_RUNNING_UNTIL_FLGUP_NODE_HPP

#include "behaviortree_cpp_v3/decorator_node.h"
#include "behaviortree_cpp_v3/basic_types.h"

namespace BT
{
/**
 * @brief A custom decorator node that continues execution until a flag is set in the Blackboard.
 */
class KeepRunningUntilFlgup : public DecoratorNode
{
public:
    // Constructor for compatibility with BehaviorTreeFactory
    KeepRunningUntilFlgup(const std::string& name, const BT::NodeConfiguration& config)
        : DecoratorNode(name, config)
    {
        setRegistrationID("KeepRunningUntilFlgup");
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<std::string>("key"),
            InputPort<std::string>("while_or_dowhile")  // ★追加
        };
    }

private:
    virtual BT::NodeStatus tick() override;
};

inline NodeStatus KeepRunningUntilFlgup::tick()
{
    setStatus(NodeStatus::RUNNING);

    // --- key取得 ---
    auto key = getInput<std::string>("key");
    if (!key)
    {
        throw RuntimeError("Missing required input [key]");
    }

    // --- モード取得 ---
    auto mode = getInput<std::string>("while_or_dowhile");
    if (!mode)
    {
        // throw RuntimeError("Missing required input [while_or_dowhile]");
        mode = ""
    }

    const std::string mode_str = mode.value();

    // --- Blackboard値取得 ---
    auto any_value = config().blackboard->getAny(key.value());

    auto checkFlagTrue = [&]() -> bool {
        if (any_value)
        {
            if (any_value->type() == typeid(bool))
            {
                return any_value->cast<bool>();
            }
            else
            {
                throw RuntimeError("Unsupported type for key [", key.value(), "]");
            }
        }
        return false;
    };

    // =========================
    // Whileモード（即終了可能）
    // =========================
    if (mode_str == "While" || mode_str == "while" )
    {
        if (checkFlagTrue())
        {
            resetChild();
            return NodeStatus::SUCCESS;
        }
    }

    // --- 子ノード実行 ---
    const NodeStatus child_state = child_node_->executeTick();

    switch (child_state)
    {
        case NodeStatus::FAILURE:
        {
            resetChild();
            return NodeStatus::FAILURE;
        }

        case NodeStatus::SUCCESS:
        {
            resetChild();

            if (checkFlagTrue())
            {
                return NodeStatus::SUCCESS;
            }

            return NodeStatus::RUNNING;
        }

        case NodeStatus::RUNNING:
        {
            return NodeStatus::RUNNING;
        }

        default:
        {
            throw LogicError("Unexpected child state");
        }
    }
}

} // namespace BT

#endif // KEEP_RUNNING_UNTIL_FLGUP_NODE_HPP