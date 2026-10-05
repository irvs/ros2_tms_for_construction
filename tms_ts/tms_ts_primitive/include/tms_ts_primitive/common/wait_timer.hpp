#ifndef WAIT_TIMER_NODE_HPP
#define WAIT_TIMER_NODE_HPP

#include "behaviortree_cpp_v3/action_node.h"

#include <chrono>
#include <string>

namespace BT
{

class WaitTimer : public StatefulActionNode
{
public:
    WaitTimer(const std::string& name,
                const NodeConfiguration& config)
      : StatefulActionNode(name, config)
    {
        setRegistrationID("wait_timer");
    }

    static PortsList providedPorts()
    {
        return {
            InputPort<double>(
                "seconds",
                1.0,
                "Wait time [s]")
        };
    }

    NodeStatus onStart() override
    {
        getInput("seconds", wait_time_);

        start_time_ = std::chrono::steady_clock::now();

        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        auto now = std::chrono::steady_clock::now();

        double elapsed =
            std::chrono::duration<double>(now - start_time_).count();

        if (elapsed >= wait_time_)
        {
            return NodeStatus::SUCCESS;
        }

        return NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        // 特に後処理なし
    }

private:
    double wait_time_;
    std::chrono::steady_clock::time_point start_time_;
};

} // namespace BT

#endif