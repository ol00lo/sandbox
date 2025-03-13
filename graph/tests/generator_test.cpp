#include "generator.hpp"
#include <catch2/catch.hpp>

using namespace g;

TEST_CASE("generator test", "[gt]")
{
    std::vector<std::vector<Tensor>> inp;
    std::vector<std::vector<Tensor>> outp;
    for (int j = 0; j < 3; ++j)
    {
        int i = j;
        inp.push_back({Tensor({double(i)})});
        outp.push_back({Tensor({double(2 * i + 2)})});
    }
    SimpleDataGenerator train_data(inp, outp);
    train_data.next_epoch(false);
    CHECK(train_data.next_input()[0][0] == 0);
    CHECK(train_data.next_gt()[0][0] == 2);
    CHECK(train_data.next_input()[0][0] == 1);
    CHECK(train_data.next_gt()[0][0] == 4);
    CHECK(train_data.is_epoch_end() == false);
    CHECK(train_data.next_input()[0][0] == 2);
    CHECK(train_data.next_gt()[0][0] == 6);
    CHECK(train_data.is_epoch_end() == true);

    train_data.next_epoch(true);
    CHECK(train_data.next_input()[0][0] != 0);
    CHECK(train_data.is_epoch_end() == false);
}