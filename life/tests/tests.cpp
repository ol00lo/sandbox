#include "driver.hpp"
#include <catch2/catch.hpp>
#include <iostream>

TEST_CASE("test1", "[1]")
{
    Driver d('c', 'c', 2, 2);
    std::vector<bool> inp = {1, 0, 0, 0};
    d.set_init_conditions(inp);
    int ret = d.start();
    CHECK(ret == 1);
    inp = {1, 1, 1, 1};
    d.set_init_conditions(inp);
    ret = d.start();
    CHECK(ret == 0);
    d.set_board_type('p');
    ret = d.start();
    CHECK(ret == 1);
}

TEST_CASE("test2", "[2]")
{
    Driver d('c', 'c', 5, 3);
    std::vector<bool> inp = {1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0};
    d.set_init_conditions(inp);
    d.set_delay(0);
    int ret = d.start();
    CHECK(ret == 8);
    d.set_board_type('m');
    ret = d.start();
    CHECK(ret == 6);
    d.set_board_type('p');
    ret = d.start();
    CHECK(ret == 5);
}