#include "graph.hpp"
#include "spdlog/sinks/stdout_color_sinks.h"

namespace
{
std::shared_ptr<spdlog::logger> logger; 
struct GraphInitializer
{
    GraphInitializer()
    {
        logger = spdlog::stdout_color_mt("graph-logger");
    }
};
GraphInitializer graph_init;
} // namespace

std::shared_ptr<spdlog::logger> g::get_logger()
{
    return logger;
}

spdlog::logger& g::log()
{
    return *logger;
}

int g::graph_ping()
{
    return 19;
}
