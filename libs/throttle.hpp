#pragma once

#include <chrono>
#include <atomic>
#include <mutex.hpp>    // spin_pause


namespace hahi {


  class pause_flag {
  public:
    void pause() const noexcept {
      paused_.store(true, std::memory_order_release);
    }

    void resume() const noexcept {
      paused_.store(false, std::memory_order_release);
      paused_.notify_all();
    }

    bool awaitable() const noexcept {
      return paused_.load(std::memory_order_acquire);
    }

    bool wait() const noexcept {
      bool was_paused = false;
      while (paused_.load(std::memory_order_acquire)) {
        paused_.wait(true, std::memory_order_relaxed);
        was_paused = true;
      }
      return was_paused;
    }

  private:
    mutable std::atomic<bool> paused_ = true;
  };


  template <typename Throttle>
  class throttle_client {
  public:
    template <typename Duration>
    throttle_client(Duration interval) noexcept :
      interval_(interval),
      tick_(0)
    {}

    void operator()() noexcept {
      if (interval_.count()) {
        if (1 == ++tick_) {
          first_ = Throttle::clock_t::now();
        }
        const auto tp = first_ + tick_ * interval_;
        last_ = Throttle::wait_until(tp);
        if (last_ > tp + interval_) {
          tick_ = 0;
        }
      }
    }

    auto interval() const noexcept {
      return interval_; 
    }

    template <typename Duration>
    void interval(const Duration& interval) {
      if (interval != interval_) {
        interval_ = interval;
        tick_ = 0;
      }
    }

  private:
    typename Throttle::duration interval_;
    typename Throttle::time_point first_;
    typename Throttle::time_point last_;
    size_t tick_;
  };


  template <
    typename Clock = std::chrono::steady_clock,
    size_t SleepThreshold = 20'000,   // go to sleep if waiting time is longer [us]
    size_t SleepBias = 16'000,        // shorten sleep interval [us]
    size_t YieldThreshold = 10,       // yield-loop as long as waiting time is (still) longer [us]
    size_t SpinThreshold = 1          // spin-loop as long as waiting time is (still) longer [us]
  >
  class throttle {
  public:
    static_assert(SleepBias < SleepThreshold, "SleepThreshold must be longer than SleepBias");
    using clock_t = Clock;
    using duration = typename clock_t::duration;
    using time_point = typename clock_t::time_point;
    using client_t = throttle_client<throttle>;

    static constexpr duration sleep_threshold() noexcept { return std::chrono::microseconds(SleepThreshold); }
    static constexpr duration sleep_bias() noexcept { return std::chrono::microseconds(SleepBias); }
    static constexpr duration yield_threshold() noexcept { return std::chrono::microseconds(YieldThreshold); }
    static constexpr duration spin_threshold() noexcept { return std::chrono::microseconds(SpinThreshold); }

    template <typename Duration>
    static time_point wait_for(const Duration& wait_duration) noexcept {
      return do_wait_for(clock_t::now(), wait_duration);
    }

    template <typename TimePoint>
    static time_point wait_until(const TimePoint& until) noexcept {
      auto now = clock_t::now();
      return do_wait_for(now, until - now);
    }

  private:
    template <typename Duration>
    static time_point do_wait_for(time_point now, const Duration& wait_duration) noexcept {
      duration dt(wait_duration);
      auto backoff = [&](auto&& fun) {
        fun();
        const auto diff = clock_t::now() - now;
        dt -= diff;
        now += diff;
      };
      if (dt > sleep_threshold()) {
        backoff([&]() { std::this_thread::sleep_for(dt - sleep_bias()); });
      }
      while (dt > yield_threshold()) {
        backoff(std::this_thread::yield);
      }
      while (dt > spin_threshold()) {
        backoff(spin_pause);
      }
      return now;
    }
  };

}
