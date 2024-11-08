#define CATCH_CONFIG_RUNNER
#include <catch2/catch.hpp>
#include "graph.hpp"

TEST_CASE("graph ping", "[ping]")
{
    CHECK(graph_ping() == 19);
}

int main(int argc, char* argv[])
{
    int result = Catch::Session().run(argc, argv);
    std::cout << "DONE" << std::endl;
    return result;
}