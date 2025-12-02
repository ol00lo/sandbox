#ifndef TASK_HPP
#define TASK_HPP
#include <algorithm>
#include <chrono>
#include <coroutine>
#include <functional>
#include <queue>
#include <vector>

class Scheduler {
public:
    static Scheduler& instance() {
        static Scheduler inst;
        return inst;
    }

    void schedule(std::coroutine_handle<> h) { ready.push(h); }

    void schedule_at(std::coroutine_handle<> h, std::chrono::steady_clock::time_point t) {
        sleepers.push_back({t, h});
        std::sort(sleepers.begin(), sleepers.end(), [](auto const& a, auto const& b) { return a.wake_time < b.wake_time; });
    }

    void run_once() {
        poll_sleepers();
        if (ready.empty())
            return;
        auto h = ready.front();
        ready.pop();
        if (!h.done())
            h.resume();
    }

    void run_some(size_t n) {
        for (size_t i = 0; i < n; ++i)
            run_once();
    }

    bool empty() {
        poll_sleepers();
        return ready.empty();
    }

    std::chrono::steady_clock::time_point now() const { return std::chrono::steady_clock::now(); }

private:
    Scheduler() = default;

    struct Sleeper {
        std::chrono::steady_clock::time_point wake_time;
        std::coroutine_handle<> h;
    };

    void poll_sleepers() {
        auto now_t = now();
        while (!sleepers.empty() && sleepers.front().wake_time <= now_t) {
            ready.push(sleepers.front().h);
            sleepers.erase(sleepers.begin());
        }
    }

    std::queue<std::coroutine_handle<>> ready;
    std::vector<Sleeper> sleepers;
};

struct YieldOnce {
    bool await_ready() const noexcept { return false; }
    void await_suspend(std::coroutine_handle<> h) const { Scheduler::instance().schedule(h); }
    void await_resume() const noexcept {}
};

struct SleepFor {
    std::chrono::milliseconds duration;
    SleepFor(std::chrono::milliseconds d) : duration(d) {}

    bool await_ready() const noexcept { return duration.count() <= 0; }

    void await_suspend(std::coroutine_handle<> h) const {
        auto t = Scheduler::instance().now() + duration;
        Scheduler::instance().schedule_at(h, t);
    }

    void await_resume() const noexcept {}
};

struct Task {
    struct promise_type {
        Task get_return_object() { return {}; }
        std::suspend_never initial_suspend() { return {}; }
        std::suspend_always final_suspend() noexcept { return {}; }
        void return_void() {}
        void unhandled_exception() { std::terminate(); }
    };
};


#endif