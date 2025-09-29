#ifndef GAME_HPP
#define GAME_HPP
#include <vector>
#include <chrono>
#include <random>

struct Lot
{
    int id;
    int price = 1;
    int winner = -1;
};

class Player
{
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
};

class GameState
{
public:
    GameState(int total_rounds, const std::vector<Player>& players);
    void run();

private:
    std::vector<Player> players;
    int total_rounds = 10;
    int current_round = 0;

    void finish() const;
};

#endif