#ifndef GRAPH_HPP
#define GRAPH_HPP
#include <iostream>
#include <memory>
#include <spdlog/spdlog.h>

namespace g
{
int graph_ping();

std::shared_ptr<spdlog::logger>& log();

class GraphInitializer
{
public:
    GraphInitializer();
};
}

#endif