
#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

class RetryManager
{
public:
    struct RetrySlot
    {
        std::atomic_bool requested{false};
        std::atomic_bool waiting{false};
    };

    static RetryManager& instance()
    {
        static RetryManager manager;
        return manager;
    }

    void registerBT(const std::string& bt_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        slots_[bt_id] = std::make_shared<RetrySlot>();
    }

    std::shared_ptr<RetrySlot> getSlot(
        const std::string& bt_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto it = slots_.find(bt_id);

        if (it == slots_.end())
        {
            return nullptr;
        }

        return it->second;
    }

    bool requestRetry(const std::string& bt_id)
    {
        auto slot = getSlot(bt_id);

        if (!slot)
        {
            return false;
        }

        if (!slot->waiting.load())
        {
            return false;
        }

        slot->requested.store(true);
        return true;
    }

    void unregisterBT(const std::string& bt_id)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        slots_.erase(bt_id);
    }

private:
    RetryManager() = default;

    std::mutex mutex_;

    std::unordered_map<
        std::string,
        std::shared_ptr<RetrySlot>
    > slots_;
};
