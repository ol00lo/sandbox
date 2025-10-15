#ifndef TASK_HPP
#define TASK_HPP

#include <coroutine>
#include <iostream>

struct Task {
    struct promise_type {
        Task get_return_object();

        std::suspend_always initial_suspend() noexcept;

        std::suspend_always final_suspend() noexcept;

        void return_void() noexcept;
        void unhandled_exception();
    };

    std::coroutine_handle<promise_type> handle;

    explicit Task(std::coroutine_handle<promise_type> h);
    Task(Task&& other) noexcept;

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    ~Task();

    void start();
    bool done() const;
};

#endif
