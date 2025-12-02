#include "player.hpp"

int Player::next_id = 0;

Player::Player(int coins, std::mt19937& gen, bool is_human) : id(next_id++), coins(coins), is_human(is_human) {
    std::uniform_real_distribution<double> d1(0.0, 1.0);
    probability = (d1(gen) * d1(gen));
    std::uniform_real_distribution<double> d2(0.5, 1.0);
    risk = d2(gen);

    last_bid_time = std::chrono::steady_clock::time_point{};
}

int Player::getId() const { return id; }
int Player::getLots() const { return lots; }
int Player::getCoins() const { return coins; }
double Player::getProbability() const { return probability; }
bool Player::isHuman() const { return is_human; }

void Player::updateBitTime() { last_bid_time = std::chrono::steady_clock::now(); }

int Player::bid(const Lot& lot, std::mt19937& gen) {
    if (last_bid_time != std::chrono::steady_clock::time_point{} &&
        std::chrono::steady_clock::now() - last_bid_time <= std::chrono::seconds(5)){
        return -1;
    }

    if (coins <= lot.price) return -1;

    int max_bid = int(std::round(coins * risk));
    int min_bid = lot.price + 1;

    if (max_bid < min_bid) return -1;

    max_bid = std::min(max_bid, coins);

    std::uniform_int_distribution<int> raise(min_bid - lot.price, max_bid - lot.price);
    return lot.price + raise(gen);
}

int Player::bid_user(int increase) {
    if (!is_human || coins <= 0) return -1;

    last_bid_time = std::chrono::steady_clock::now();
    return increase;
}

void Player::buy(const Lot& lot) {
    if (coins >= lot.price) {
        coins -= lot.price;
        ++lots;
    }
}
