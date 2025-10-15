#include "awaiter.hpp"

bool SleepAwaiter::await_ready() const noexcept { return duration.count() <= 0; }

void SleepAwaiter::await_suspend(std::coroutine_handle<> h) const {
    std::thread([h, this]() {
        std::this_thread::sleep_for(duration);
        h.resume();
    }).detach();
}

void SleepAwaiter::await_resume() const noexcept {}
