#include <iostream>
#include <vector>

#include "awaiter.hpp"
#include "game.hpp"
#include "task.hpp"
#include <mutex>

using namespace std::chrono_literals;

Task run_bidder(Player& player, Lot& lot, bool& finished, std::mutex& lot_mutex,
                std::chrono::steady_clock::time_point& last_bid_time) {
    std::mt19937 gen(std::random_device{}());
    std::uniform_int_distribution<int> delay_dist(300, 2000);

    while (!finished) {
        co_await sleep_for(std::chrono::milliseconds(delay_dist(gen)));

        if (finished) co_return;

        int new_price = player.bid(lot, gen);
        {
            std::lock_guard lock(lot_mutex);
            if (finished) co_return;

            if (new_price > lot.price) {
                lot.price = new_price;
                lot.winner = player.getId();
                last_bid_time = std::chrono::steady_clock::now();
                std::cout << "Player " << player.getId() << " bid " << new_price << "\n";
            }
        }
    }
}

Task run_auction_round(int round_id, std::vector<Player>& players) {
    using namespace std::chrono_literals;

    std::cout << "\n=== Round " << round_id << " ===\n";

    Lot lot{round_id, 0, -1};
    bool finished = false;
    std::mutex lot_mutex;
    auto last_bid_time = std::chrono::steady_clock::now();

    std::vector<Task> bidder_tasks;
    bidder_tasks.reserve(players.size());
    for (auto& p : players) {
        bidder_tasks.emplace_back(run_bidder(p, lot, finished, lot_mutex, last_bid_time));
        bidder_tasks.back().start();
    }

    while (!finished) {
        co_await sleep_for(500ms);

        auto now = std::chrono::steady_clock::now();
        if (now - last_bid_time > 4s) {
            std::lock_guard lock(lot_mutex);
            finished = true;
        }
    }

    if (lot.winner != -1) {
        players[lot.winner].buy(lot);
        std::cout << "Winner: Player " << lot.winner << " with price " << lot.price << "\n";
    } else {
        std::cout << "No bids this round.\n";
    }

    co_return;
}

int main() {
    std::vector<Player> players;
    for (int i = 0; i < 10; ++i) players.emplace_back(10);

    for (int round = 1; round <= 15; ++round) {
        auto task = run_auction_round(round, players);
        task.start();
        while (!task.done()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    std::cout << "=== Game over ===\n";
}