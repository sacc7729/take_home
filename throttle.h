// Rate Limiter for a robot command.

// Makes some assumptions:
// * Only one process is allowed to communicate with the robot
//   at a time.  If multiple processes interfaced with the robot
//   we would need to develop IPCs to coordinate when certain
//   processes get to interface with the robot.
// * start(), stop(), and submit() are all protected by the same mutex.
//   The order of precedence is stop, start, then submit.
// * We only care about the latest command that is submitted.
//   If a command is submitted, no previously submitted commands will be
//   sent to the robot.
// * The RobotThrottle does not allow submits when the RobotThrottle
//   is not running.

# pragma once

#include "priority_lock.h"
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <thread>

namespace robot_helpers {

    class RobotThrottle {

        public:
            using Clock = std::chrono::steady_clock;
            using NowFunction = std::function<Clock::time_point()>;

            // The constructor
            // Takes in a function and forces command to be executed at
            // or below the rate_hz.
            explicit RobotThrottle(
                std::function<void(double)> send_func, double rate_hz = 100.0,
                NowFunction now = [] { return Clock::now(); });

            // We don't really need copies and moves.
            RobotThrottle(const RobotThrottle&) = delete;
            RobotThrottle(const RobotThrottle&&) = delete;
            RobotThrottle& operator=(const RobotThrottle&) = delete;
            RobotThrottle&& operator=(const RobotThrottle) = delete;

            ~RobotThrottle();

            // Start sending commands based on send_func.
            // Commands sent at rate_hz frequency.
            void start();

            // Stops sending commands to the robot.
            // Prevents the queueing of commands.
            void stop();

            // Submit command for execution on the robot.
            // Overwrites any commands that have not yet been set to the robot.
            // Note: As soon as the RobotThrottle is constructed, the quota
            //       is available to send commands at the rate of rate_hz.
            void submit(double command);

        private:
            // Controlled by start() and stop().
            bool is_running_;

            // Blocks until there is a
            // * command submitted
            // * is_running_ is true
            // * command has not been send to robot in 1/rate_hz sec.
            // 
            // Runs send_func_ if above conditions are met.
            void run();

            // Continually runs run() in a new thread.
            std::thread worker_;
            // Whether to continually run run();
            bool shutting_down_ = false;

            // Set by submit().
            std::optional<double> command_;
            // Lock which provides linearizability.
            // Order of precedence is stop, start, then submit.
            PriorityLock priority_lock_;
            

            // Notifies the worker_ that is_running_ is true
            // or that command_ is set.  This is useful because we
            // can 'pause' worker_.run() until necessary conditions are met.
            std::condition_variable cv_;
            // Guards the conditions needed for cv_
            std::mutex condition_mutex_;

            // The function to run.
            std::function<void(double)> send_func_;
            // Helpful for mocking the clock.
            NowFunction now_;
            // The time period we wait for to send a command.
            double period_seconds_;
            // Internally used to see if a command can be sent.
            Clock::time_point next_allowed_;
    };
}
