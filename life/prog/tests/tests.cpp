#include "arguments.hpp"
#include "driver.hpp"
#include "global.hpp"
#include <catch2/catch.hpp>
#include <iostream>

TEST_CASE("test1", "[1]")
{
    global_argc = 3;
    global_argv = new char*[global_argc];

    global_argv[0] = strdup("exe");
    global_argv[1] = strdup("--initfile");
    global_argv[2] = strdup("file.txt");

    Arguments argum(global_argc, global_argv);
    std::cout << *global_argv[2] << std::endl;
    CHECK(argum.height == 5);
    CHECK(argum.width == 3);
    Driver d(argum);
    int ret = d.start();
    CHECK(ret == 8);
}
TEST_CASE("test2", "[2]")
{
    global_argc = 5;
    global_argv = new char*[global_argc];

    global_argv[0] = ("exe");
    global_argv[1] = ("--dim");
    global_argv[2] = ("2x2");
    global_argv[3] = ("--random");
    global_argv[4] = ("16");

    Arguments argum(global_argc, global_argv);
    std::cout << *global_argv[2] << std::endl;
    CHECK(argum.height == 2);
    CHECK(argum.width == 2);
    Driver d(argum);
    int ret = d.start();
    CHECK(ret == 0);
}