
#pragma once

#include <string>
#include <iostream>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "tms_ts_primitive/common/retry_manager.hpp"

class ManualRetry : public BT::DecoratorNode
{
public:
    ManualRetry(
        const std::string& name,
        const BT::NodeConfiguration& config)
        : BT::DecoratorNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("bt_id")
        };
    }

private:
    bool waiting_ = false;

    BT::NodeStatus tick() override
    {
        std::string bt_id;

        if (!getInput("bt_id", bt_id))
        {
            throw BT::RuntimeError(
                "ManualRetry: bt_id is required");
        }

        auto slot =
            RetryManager::instance().getSlot(bt_id);

        if (!slot)
        {
            throw BT::RuntimeError(
                "ManualRetry: BT ID not registered: ",
                bt_id);
        }

        // 再試行待機中
        if (waiting_)
        {
            if (!slot->requested.exchange(false))
            {
                return BT::NodeStatus::RUNNING;
            }

            waiting_ = false;
            slot->waiting.store(false);

            std::cout
                << "[ManualRetry] Retry: "
                << bt_id << std::endl;
        }

        // 子ノードを実行
        BT::NodeStatus result =
            child_node_->executeTick();

        if (result == BT::NodeStatus::FAILURE)
        {
            std::cout
                << "[ManualRetry] FAILURE: "
                << bt_id << std::endl;

            // 子ノードをリセット
            resetChild();

            waiting_ = true;
            slot->waiting.store(true);

            return BT::NodeStatus::RUNNING;
        }

        return result;
    }

    void halt() override
    {
        std::string bt_id;

        if (getInput("bt_id", bt_id))
        {
            auto slot =
                RetryManager::instance().getSlot(bt_id);

            if (slot)
            {
                slot->waiting.store(false);
                slot->requested.store(false);
            }
        }

        waiting_ = false;

        BT::DecoratorNode::halt();
    }
};
