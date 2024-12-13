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
        return { InputPort<std::string>("key") };
    }

private:
    virtual BT::NodeStatus tick() override;
};

inline NodeStatus KeepRunningUntilFlgup::tick()
{
    setStatus(NodeStatus::RUNNING);

    Optional<std::string> key = getInput<std::string>("key");
    if (!key)
    {
        throw RuntimeError("Missing required input [key]");
    }

    const NodeStatus child_state = child_node_->executeTick();
    auto any_value = config().blackboard->getAny(key.value());

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
            if(any_value)
            {
                if (any_value->type() == typeid(bool))
                {
                    bool value = any_value->cast<bool>();
                    if (value)
                    {
                        return NodeStatus::SUCCESS;
                    }
                }
                else
                {
                    throw RuntimeError("Unsupported type for key [", key.value(), "]");
                }
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
    return status();
}
} // namespace BT

#endif // KEEP_RUNNING_UNTIL_FLGUP_NODE_HPP
