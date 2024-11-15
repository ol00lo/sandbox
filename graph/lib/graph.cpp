#include "graph.hpp"
#include "spdlog/sinks/stdout_color_sinks.h"

static std::shared_ptr<spdlog::logger> logger;
g::GraphInitializer::GraphInitializer()
{
    logger = spdlog::stdout_color_mt("graph-logger");
}
std::shared_ptr<spdlog::logger>& g::log()
{
    return logger;
}
static g::GraphInitializer graph_init;

int g::graph_ping()
{
    return 19;
}
