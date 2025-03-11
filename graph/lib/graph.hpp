#ifndef GRAPH_HPP
#define GRAPH_HPP
#include <iostream>
#include <stdexcept>
#define _THROW_NOT_IMP_ throw std::runtime_error("Not implemented");
#include <memory>
#include <spdlog/spdlog.h>

namespace g
{
int graph_ping();
spdlog::logger& log();
std::shared_ptr<spdlog::logger> get_logger();
}

#endif