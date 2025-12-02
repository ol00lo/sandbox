#ifndef TASK_HPP
#define TASK_HPP

#include <chrono>
#include <coroutine>
#include <queue>
#include <vector>

class Scheduler
{
public:
    static Scheduler& instance();

    void schedule(std::coroutine_handle<> h);
    void schedule_at(std::coroutine_handle<> h, std::chrono::steady_clock::time_point t);

    void run_once();
    void run_some(size_t n);
    bool empty() const;
    std::chrono::steady_clock::time_point now() const;

private:
    Scheduler();
    struct Sleeper
    {
        std::chrono::steady_clock::time_point wake_time;
        std::coroutine_handle<> h;
    };
    void poll_sleepers();

    std::queue<std::coroutine_handle<>> ready;
    std::vector<Sleeper> sleepers;
};

struct YieldOnce
{
    bool await_ready() const noexcept;
    void await_suspend(std::coroutine_handle<> h) const;
    void await_resume() const noexcept;
};

struct SleepFor
{
    std::chrono::milliseconds duration;
    SleepFor(std::chrono::milliseconds d);
    bool await_ready() const noexcept;
    void await_suspend(std::coroutine_handle<> h) const;
    void await_resume() const noexcept;
};

struct Task
{
    struct promise_type
    {
        Task get_return_object();
        std::suspend_never initial_suspend();
        std::suspend_always final_suspend() noexcept;
        void return_void();
        void unhandled_exception();
    };
};

#endif