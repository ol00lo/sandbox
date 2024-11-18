#define CATCH_CONFIG_RUNNER
#include <catch2/catch.hpp>

TEST_CASE("graph ping", "[ping]")
{
    CHECK(19 == 19);
}

int main(int argc, char* argv[])
{
    int result = Catch::Session().run(argc, argv);
    std::cout << "DONE" << std::endl;
    return result;
}