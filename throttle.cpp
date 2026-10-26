#include "throttle.h"

#include <exception>
#include <functional>
#include <iostream>
#include <utility>
#include <chrono>

namespace robot_helpers {

  // TODO: We need to have a better injected clock here.
  //       Right now we have two sources of time.
  //       The 'target time' is the time we are trying to reach in order
  //       to send the next command (which is now() + period_seconds_).
  //       The 'actual time' is the time that is being compared against.
  //       When actual time is greater than or equal to target time, then
  //       the send quota is refilled.
  //       What we actually need here is a way to mock 'actual time' and not
  //       'target time'.
  //       When this is fixed, we can re-enable the rate_limit test.
  RobotThrottle::RobotThrottle(std::function<void(double)> send_func, double rate_hz,
                               NowFunction now) :
  send_func_(std::move(send_func)),
  now_(std::move(now)) {
    period_seconds_ = 1.0 / rate_hz;
    next_allowed_ = now_();
    is_running_ = false;
    // Continually runs the run sequence on a separate thread
    worker_ = std::thread([this] {
        while (!shutting_down_) {
          run(); 
        }
      });
  }

  RobotThrottle::~RobotThrottle() {
    {
      std::lock_guard<std::mutex> lk(condition_mutex_);
      shutting_down_ = true;
    }
    cv_.notify_one();
    worker_.join();
  }

  void RobotThrottle::submit(double command) {
    priority_lock_.lock(2);
    if (!is_running_) {
      priority_lock_.unlock();
      throw std::logic_error("Unable to submit "
        "command when throttle is not running.");
    }
    {
      std::lock_guard<std::mutex> lk(condition_mutex_);
      command_ = command;
    }
    cv_.notify_one();
    priority_lock_.unlock();
  }

  void RobotThrottle::start() {
    priority_lock_.lock(1);
    {
      std::lock_guard<std::mutex> lk(condition_mutex_);
      is_running_ = true;
    }
    cv_.notify_one();
    priority_lock_.unlock();
  }

  void RobotThrottle::stop() {
    priority_lock_.lock(0);
    {
      std::lock_guard<std::mutex> lk(condition_mutex_);
      is_running_ = false;
      command_.reset();
    }
    priority_lock_.unlock();
  }

  void RobotThrottle::run() {
    std::unique_lock<std::mutex> lk(condition_mutex_);
    // Only progress if shutting down or
    // quota is available.
    cv_.wait_until(lk, next_allowed_, [this] {
      return shutting_down_;
    });
    // Progress if preconditions met.
    cv_.wait(lk, [this] {
      return shutting_down_ || (command_.has_value() && is_running_);
    });
    if (shutting_down_) {
      return;
    }

    const Clock::time_point now = now_();

    next_allowed_ = now + std::chrono::duration_cast<Clock::duration>(
                                std::chrono::duration<double>(period_seconds_));
    const double command = command_.value();
    command_.reset();
    try {
      send_func_(command);
    } catch (const std::exception& e) {
      std::cout << "some error happened: " << e.what() << "\n";
    }
  }
} 
