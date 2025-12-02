#ifndef PLAYER_HPP
#define PLAYER_HPP

#include <chrono>
#include <random>

struct Lot
{
    int id;
    int price = 0;
    int winner = -1;
};

class Player
{
public:
    Player(int coins, std::mt19937& gen, bool is_human = false);

    int bid(const Lot& lot, std::mt19937& gen);
    int bid_user(int increase);
    void buy(const Lot& lot);

    int getId() const;
    int getLots() const;
    int getCoins() const;
    double getProbability() const;
    void updateBitTime();
    bool isHuman() const;

    std::chrono::steady_clock::time_point last_bid_time;
private:
    static int next_id;
    int id;
    int coins;
    int lots = 0;
    bool is_human;
    double probability;
    double risk;
};

#endif