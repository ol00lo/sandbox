#include "graph.hpp"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace
{
std::shared_ptr<spdlog::logger> logger_;
struct GraphInitializer
{
    GraphInitializer()
    {
        logger_ = spdlog::stdout_color_mt("graph-logger");
    }
};
GraphInitializer graph_init;
} // namespace

std::shared_ptr<spdlog::logger> g::logger()
{
    return logger_;
}

spdlog::logger& g::log()
{
    return *logger_;
}

int g::graph_ping()
{
    return 19;
}
