#ifndef TASK_HPP
#define TASK_HPP

#include <coroutine>
#include <queue>

class Scheduler {
public:
    static Scheduler& instance() {
        static Scheduler inst;
        return inst;
    }

    void schedule(std::coroutine_handle<> h) { tasks.push(h); }
    void run() {
        while (!tasks.empty()) {
            auto h = tasks.front();
            tasks.pop();
            h();
        }
    }
    bool empty() const { return tasks.empty(); }


private:
    std::queue<std::coroutine_handle<>> tasks;
};

struct YieldOnce {
    bool await_ready() const noexcept { return false; }
    void await_suspend(std::coroutine_handle<> h) const { Scheduler::instance().schedule(h); }
    void await_resume() const noexcept {}
};

struct Task
{
    struct promise_type {
        Task get_return_object() { return {}; }

        std::suspend_never initial_suspend() { return {}; }
        std::suspend_never final_suspend() noexcept { return {}; }

        void return_void() {}
        void unhandled_exception() { std::terminate(); }
    };
};

#endif