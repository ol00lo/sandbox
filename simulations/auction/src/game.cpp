#include "game.hpp"

#include <iostream>

int Player::next_id = 0;

Player::Player(int coins) : coins(coins), id(next_id++) {}

int Player::bid(const Lot& lot, std::mt19937& gen) const {
    int max_raise = coins - lot.price;

    if (max_raise <= 0) return lot.price;

    std::uniform_int_distribution<int> dist(1, max_raise);

    return lot.price + dist(gen);
}

void Player::buy(const Lot& lot) {
    coins -= lot.price;
    if (coins < 0) throw std::runtime_error("Player has negative coins.");
    lots++;
}

int Player::getId() const { return id; }

int Player::getLots() const { return lots; }
