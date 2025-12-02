#include "task.hpp"
#include <algorithm>
#include <coroutine>
#include <iostream>
#include <stdexcept>

Scheduler& Scheduler::instance() {
    static Scheduler inst;
    return inst;
}

Scheduler::Scheduler() = default;

void Scheduler::schedule(std::coroutine_handle<> h) {
    ready.push(h);
}

void Scheduler::schedule_at(std::coroutine_handle<> h, std::chrono::steady_clock::time_point t) {
    sleepers.push_back({t, h});
    std::sort(sleepers.begin(), sleepers.end(), [](auto const& a, auto const& b) { return a.wake_time < b.wake_time; });
}

void Scheduler::poll_sleepers() {
    auto now_t = now();
    while (!sleepers.empty() && sleepers.front().wake_time <= now_t) {
        ready.push(sleepers.front().h);
        sleepers.erase(sleepers.begin());
    }
}

void Scheduler::run_once() {
    poll_sleepers();
    if (ready.empty()) return;

    auto h = ready.front();
    ready.pop();
    if (!h.done())
        h.resume();
}

void Scheduler::run_some(size_t n) {
    for (size_t i = 0; i < n; ++i)
        run_once();
}

bool Scheduler::empty() const {
    const_cast<Scheduler*>(this)->poll_sleepers();
    return ready.empty();
}

std::chrono::steady_clock::time_point Scheduler::now() const {
    return std::chrono::steady_clock::now();
}


bool YieldOnce::await_ready() const noexcept { return false; }
void YieldOnce::await_suspend(std::coroutine_handle<> h) const { Scheduler::instance().schedule(h); }
void YieldOnce::await_resume() const noexcept {}


SleepFor::SleepFor(std::chrono::milliseconds d) : duration(d) {}
bool SleepFor::await_ready() const noexcept { return duration.count() <= 0; }
void SleepFor::await_suspend(std::coroutine_handle<> h) const {
    auto t = Scheduler::instance().now() + duration;
    Scheduler::instance().schedule_at(h, t);
}
void SleepFor::await_resume() const noexcept {}


Task Task::promise_type::get_return_object() { return {}; }
std::suspend_never Task::promise_type::initial_suspend() { return {}; }
std::suspend_always Task::promise_type::final_suspend() noexcept { return {}; }
void Task::promise_type::return_void() {}
void Task::promise_type::unhandled_exception() { std::terminate(); }
