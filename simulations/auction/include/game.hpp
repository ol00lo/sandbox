#ifndef GAME_HPP
#define GAME_HPP
#include <chrono>
#include <random>
#include <vector>
#include "task.hpp"

struct Lot {
    int id;
    int price = 0;
    int winner = -1;
};

class Player {
   public:
    Player(int coins, std::mt19937& gen);

    int bid(const Lot& lot, std::mt19937& gen);
    void buy(const Lot& lot);

    int getId() const;
    int getLots() const;
    int getCoins() const;
    double getProbability() const;
    void updateBitTime();
    std::chrono::steady_clock::time_point last_bid_time;

   private:
    static int next_id;
    int id;
    int coins;
    int lots = 0;

    double probability;
    double risk;
};


Task bidder(Player& p, Lot& lot, std::mt19937& gen, bool& someone_bid);

void run_round(std::vector<Player>& players, int lot_id, std::mt19937& gen);

#endif