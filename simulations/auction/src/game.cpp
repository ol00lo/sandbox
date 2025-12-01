#include "game.hpp"
#include <iostream>

int Player::next_id = 0;

Player::Player(int coins, std::mt19937& gen) : id(next_id++), coins(coins)
{
    std::uniform_real_distribution<double> dist(0.0, 1.0);
    probability = (dist(gen) * dist(gen));
    std::uniform_real_distribution<double> dist2(0.1, 1.0);
    risk = dist2(gen);
}

int Player::getId() const { return id; }
int Player::getLots() const { return lots; }
int Player::getCoins() const { return coins; }
double Player::getProbability() const { return probability; }

void Player::updateBitTime()
{
    last_bid_time = std::chrono::steady_clock::now();
}

int Player::bid(const Lot& lot, std::mt19937& gen) const {
    if (coins <= lot.price || std::chrono::steady_clock::now() - last_bid_time <= std::chrono::seconds(5))
        return -1;

    int max_bid = int(std::round(coins*risk));
    int min_bid = lot.price + 1;

    if (max_bid < min_bid)
    {
        std::cout << "Player " << id << " dont want risk\n";
        return -1;
    }

    max_bid = std::min(max_bid, coins);

    std::uniform_int_distribution<int> raise(min_bid - lot.price, max_bid - lot.price);
    return lot.price + raise(gen);
}

void Player::buy(const Lot& lot) {
    if (coins >= lot.price) {
        coins -= lot.price;
        ++lots;
    }
}


Task bidder(Player& p, Lot& lot, std::mt19937& gen, bool& someone_bid) {
    std::uniform_real_distribution<double> prob(0.0, 1.0);

    while (true) {
        if (p.getCoins() <= lot.price)
            co_return;
        if (prob(gen) < p.getProbability()) {
            int new_price = p.bid(lot, gen);
            if (new_price != -1) {
                lot.price = new_price;
                lot.winner = p.getId();
                someone_bid = true;
                p.updateBitTime();
                std::cout << "Player " << p.getId() << " bids " << new_price << "\n";
            }
            else
            {
                co_return;
            }
        }

        co_await YieldOnce{};
    }
}

void run_round(std::vector<Player>& players, int lot_id, std::mt19937& gen) {
    std::cout << "\n=== Round " << lot_id << " ===\n";

    Lot lot;
    lot.id = lot_id;

    bool someone_bid = false;

    for (auto& p : players)
    {
        bidder(p, lot, gen, someone_bid);
    }

    auto silence_start = std::chrono::steady_clock::now();
    while (true)
    {
        someone_bid = false;

        Scheduler::instance().run();

        if (someone_bid)
            silence_start = std::chrono::steady_clock::now();
        if (std::chrono::steady_clock::now() - silence_start >= std::chrono::seconds(4))
            break;
    }

    if (lot.winner != -1)
    {
        std::cout << "Winner: Player " << lot.winner << " for price " << lot.price << "\n";
        players[lot.winner].buy(lot);
    }
    else
    {
        std::cout << "No bids. Nobody wins.\n";
    }
}