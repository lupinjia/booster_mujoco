#pragma once

// A minimal std::thread-based recurrent thread, replacing the
// unitree::common::RecurrentThread used by unitree_mujoco. The
// booster_robotics_sdk does not ship such a utility.
//
// Usage (same as unitree::common::RecurrentThread):
//   auto th = std::make_shared<RecurrentThread>("bridge", 1, 2000,
//               std::bind(&Foo::run, this));

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace booster
{
namespace common
{

class RecurrentThread
{
public:
    using Callback = std::function<void()>;

    RecurrentThread(const std::string& name, int priority, int interval_us, Callback callback)
        : name_(name), interval_us_(interval_us), callback_(std::move(callback)), stop_(false)
    {
        thread_ = std::thread(&RecurrentThread::loop, this);
    }

    ~RecurrentThread()
    {
        stop_ = true;
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    RecurrentThread(const RecurrentThread&) = delete;
    RecurrentThread& operator=(const RecurrentThread&) = delete;

    void Stop()
    {
        stop_ = true;
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

private:
    void loop()
    {
        const auto interval = std::chrono::microseconds(interval_us_);
        auto next = std::chrono::steady_clock::now();
        while (!stop_)
        {
            try
            {
                callback_();
            }
            catch (const std::exception& e)
            {
                std::cerr << "[" << name_ << "] RecurrentThread callback error: " << e.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "[" << name_ << "] RecurrentThread callback unknown error" << std::endl;
            }
            next += interval;
            std::this_thread::sleep_until(next);
        }
    }

    std::string name_;
    int interval_us_;
    Callback callback_;
    std::atomic<bool> stop_;
    std::thread thread_;
};

using RecurrentThreadPtr = std::shared_ptr<RecurrentThread>;

} // namespace common
} // namespace booster
