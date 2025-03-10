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

void g::set_log_debug()
{
    logger->set_level(spdlog::level::debug);
}

spdlog::logger& g::log()
{
    return *logger;
}

int g::graph_ping()
{
    return 19;
}
