#include "simlife_c_api.h"
#include <catch2/catch.hpp>

TEST_CASE("C-API for Driver", "[3]")
{
    void* driver;
    int r;
    REQUIRE(driver_create('c', 'c', 5, 3, &driver) == 0);
    int inp[] = {1, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0};
    REQUIRE(driver_set_init_conditions(driver, inp, 15) == 0);
    driver_set_delay(driver, 0);
    driver_start(driver, &r);
    CHECK(r == 8);
    driver_set_board_type(driver, 'm');
    driver_start(driver, &r);
    CHECK(r == 6);
    driver_set_board_type(driver, 'p');
    driver_start(driver, &r);
    CHECK(r == 5);
    CHECK(driver_destroy(driver) == 0);
}