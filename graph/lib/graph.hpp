#ifndef GRAPH_HPP
#define GRAPH_HPP
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>
#include <stdexcept>

#define _THROW_NOT_IMP_ throw std::runtime_error("Not implemented");

namespace g
{
int graph_ping();
spdlog::logger& log();
std::shared_ptr<spdlog::logger> logger();
} // namespace g

#endif