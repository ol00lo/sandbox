#include "timer.hpp"
#include <chrono>

Timer::Timer()
{
    _pimpl = std::make_shared<Impl>();
}

class Timer::Impl
{
public:
    void start()
    {
        _t = std::chrono::high_resolution_clock().now();
    }

    int end()
    {
        auto t = std::chrono::high_resolution_clock().now();
        return std::chrono::duration_cast<std::chrono::milliseconds>(t - _t).count();
    }

private:
    std::chrono::high_resolution_clock::time_point _t;
};

void Timer::start()
{
    _pimpl->start();
}

int Timer::end()
{
    return _pimpl->end();
}