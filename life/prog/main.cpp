#define CATCH_CONFIG_RUNNER
#include "global.hpp"
#include <catch2/catch.hpp>

int global_argc;
char** global_argv;

int main(int argc, char* argv[])
{
    global_argc = argc;
    global_argv = argv;
    int result = Catch::Session().run();
    std::cout << "DONE" << std::endl;
    return result;
}