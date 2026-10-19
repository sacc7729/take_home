#include "throttle.h"

#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <mutex>
#include <stdexcept>
#include <utility>
#include <vector>

using TestClock = std::chrono::steady_clock;
using namespace std::chrono_literals;

namespace {

// Used to record which commands were sent to the robot
// and the time that they were sent.
struct SendRecorder {
  std::mutex mutex;
  std::condition_variable changed;
  std::vector<std::pair<TestClock::time_point, double>> calls;

  void operator()(double command) {
    std::lock_guard<std::mutex> lock(mutex);
    calls.emplace_back(TestClock::now(), command);
    changed.notify_one();
  }

  // Returns true if the timeout did not expire.
  // Prevents tests from hanging if the size hasn't been reached.
  //
  // Just going to wait 2000ms and be lazy, since there aren't many tests.
  bool wait_for_size(std::size_t expected,
                    std::chrono::milliseconds timeout = 2000ms) {
    std::unique_lock<std::mutex> lock(mutex);
    return changed.wait_for(lock, timeout,
                            [&] { return calls.size() >= expected; });
  }

  std::vector<std::pair<TestClock::time_point, double>> snapshot() {
    std::lock_guard<std::mutex> lock(mutex);
    return calls;
  }
};

struct MockClock {
  TestClock::time_point current = TestClock::time_point{};

  TestClock::time_point now() {
    return current;
  }

  void advance(TestClock::duration amount) {
    current += amount;
  }
};

bool test_single_command() {
  SendRecorder recorder;
  robot_helpers::RobotThrottle throttle(
      [&](double command) { recorder(command); });
  throttle.start();
  throttle.submit(0.5);

  const bool sent = recorder.wait_for_size(1);
  const auto calls = recorder.snapshot();
  throttle.stop();
  return sent && calls.size() == 1 && calls.front().second == 0.5;
}

bool test_latest_command() {
  SendRecorder recorder;
  MockClock clock;
  robot_helpers::RobotThrottle throttle(
      [&](double command) { recorder(command); }, 100.0,
      [&] { return clock.now(); });
  throttle.start();

  throttle.submit(2.0);
  throttle.submit(3.0);
  throttle.submit(4.0);
  clock.advance(10ms);
  const bool sent_latest = recorder.wait_for_size(1);
  const auto calls = recorder.snapshot();
  throttle.stop();
  return sent_latest && calls.size() == 1 && calls[0].second == 4.0;
}

bool test_stop_discards_pending() {
  SendRecorder recorder;
  MockClock clock;
  robot_helpers::RobotThrottle throttle(
      [&](double command) { recorder(command); }, 100.0,
      [&] { return clock.now(); });
  throttle.start();
  throttle.submit(2.0);
  throttle.stop();
  const auto calls = recorder.snapshot();
  return calls.size() == 0;
}

bool test_rate_limit() {
  SendRecorder recorder;
  MockClock clock;
  robot_helpers::RobotThrottle throttle(
      [&](double command) { recorder(command); }, 100.0,
      [&] { return clock.now(); });

  throttle.start();
  throttle.submit(1.0);
  if (!recorder.wait_for_size(1)) {
    throttle.stop();
    return false;
  }

  throttle.submit(2.0);
  const bool blocked_before_period =
      !recorder.wait_for_size(2);
  if (!blocked_before_period) {
    throttle.stop();
    return false;
  }

  clock.advance(10ms);
  throttle.submit(3.0);
  const bool sent_after_period = recorder.wait_for_size(2);
  const auto calls = recorder.snapshot();
  throttle.stop();

  return sent_after_period && calls.size() == 2 &&
         calls[0].second == 1.0 && calls[1].second == 3.0;
}

bool test_submit_requires_start() {
  robot_helpers::RobotThrottle throttle(
      [](double) {}, 100.0, [] { return TestClock::now(); });

  try {
    throttle.submit(42.0);
  } catch (const std::logic_error&) {
    return true;
  } catch (...) {
    return false;
  }
  return false;
}

}  // namespace

int main() {
  // pair of test name and function pointer to test
  const std::pair<const char*, bool(*)()> tests[] = {
      {"single_command", test_single_command},
      {"latest_command",
       test_latest_command},
      {"stop_discards_pending", test_stop_discards_pending},
      {"rate_limit", test_rate_limit},
      {"submit_requires_start", test_submit_requires_start},
  };

  int failures = 0;
  for (const auto& test : tests) {
    const bool passed = test.second();
    std::printf("[%s] %s\n", passed ? "PASS" : "FAIL", test.first);
    failures += passed ? 0 : 1;
  }
  return failures == 0 ? 0 : 1;
}