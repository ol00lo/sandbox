#include "simlife_c_api.h"
#include <catch2/catch.hpp>

TEST_CASE("C-API for Driver", "[3]")
{
    void* driver;
    REQUIRE(driver_create('c', 'c', 5, 3, &driver) == 0);
    int inp[] = {1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0};
    REQUIRE(driver_set_init_conditions(driver, inp, 15) == 0);
    driver_set_delay(driver, 0);
    CHECK(driver_start(driver) == 8);
    driver_set_board_type(driver, 'm');
    CHECK(driver_start(driver) == 6);
    driver_set_board_type(driver, 'p');
    CHECK(driver_start(driver) == 5);
    CHECK(driver_destroy(driver) == 0);
}