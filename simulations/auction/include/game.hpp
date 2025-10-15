#ifndef GAME_HPP
#define GAME_HPP
#include <chrono>
#include <random>
#include <vector>

struct Lot {
    int id;
    int price = 1;
    int winner = -1;
};

class Player {
   public:
    Player(int coins);
    int bid(const Lot& lot, std::mt19937& gen) const;
    void buy(const Lot& lot);
    int getId() const;
    int getLots() const;

   private:
    static int next_id;
    int id;
    int coins;
    int lots = 0;
    std::chrono::steady_clock::time_point last_bid_time = std::chrono::steady_clock::now();
    bool is_human = false;
};

#endif