#pragma once

#include <array>
#include <condition_variable>
#include <mutex>

namespace robot_helpers {

    class PriorityLock {
    public:
        // priority 0 = highest, 2 = lowest
        void lock(int prio) {
            std::unique_lock<std::mutex> lk(m_);
            ++waiting_[prio];
            cv_.wait(lk, [&] { return !busy_ && !higherWaiting(prio); });
            --waiting_[prio];
            busy_ = true;
        }

        void unlock() {
            {
                std::lock_guard<std::mutex> lk(m_);
                busy_ = false;
            }
            cv_.notify_all();
        }

    private:
        bool higherWaiting(int prio) const {
            for (int i = 0; i < prio; ++i)
                if (waiting_[i] > 0) {
                    return true;
                }
            return false;
        }

        std::mutex m_;
        std::condition_variable cv_;
        std::array<int, 3> waiting_{};
        bool busy_ = false;
    };
}