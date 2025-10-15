#ifndef AWAITER_HPP
#define AWAITER_HPP
#include <chrono>
#include <coroutine>
#include <thread>

struct SleepAwaiter {
    std::chrono::milliseconds duration;
    bool await_ready() const noexcept;

    void await_suspend(std::coroutine_handle<> h) const;

    void await_resume() const noexcept;
};

inline SleepAwaiter sleep_for(std::chrono::milliseconds d) { return {d}; }

#endif