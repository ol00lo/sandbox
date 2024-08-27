#ifndef TIMER_H
#define TIMER_H
#include <iostream>

class Timer
{
public:
    Timer();
    void start();
    int end();

private:
    class Impl;
    std::shared_ptr<Impl> _pimpl;
};

#endif