#include "game.hpp"
#include <iostream>


int Player::next_id = 0;

Player::Player(int coins) : coins(coins), id(next_id++) {}

int Player::bid(const Lot& lot, std::mt19937& gen) const
{
    int max_raise = coins - lot.price;

    if (max_raise <= 0)
        return lot.price;

    std::uniform_int_distribution<int> dist(1, max_raise);

    int raise = dist(gen);
    return lot.price + raise;
}

void Player::buy(const Lot& lot)
{
	coins -= lot.price;
    if (coins < 0)
		throw std::runtime_error("Player has negative coins.");
    lots++;
}

int Player::getId() const
{
    return id;
}

int Player::getLots() const
{
    return lots;
}

GameState::GameState(int total_rounds, const std::vector<Player>& players) : total_rounds(total_rounds), players(players) {}

void GameState::run()
{
    std::mt19937 rnd(std::random_device{}());
    std::uniform_int_distribution<int> coin_flip(0, 1);

    while (current_round < total_rounds)
    {
        current_round++;

        std::cout << "Round " << current_round << std::endl;
        Lot lot{current_round};
        
        for (int i = 0; i < players.size(); i++)
        {

			if (coin_flip(rnd))
			{
				int new_price = players[i].bid(lot, rnd);
                if (new_price > lot.price)
                {
					lot.price = new_price;
                    lot.winner = i;
					std::cout<<"Player " << players[i].getId() << " bid " << new_price << "\n";
                }
                else
                {
                    std::cout<<"Player " << players[i].getId() << " don't have enough coins.\n";
                }
			}
            else
            {
                std::cout<<"Player " << players[i].getId() << " passed.\n";
            }
        }
        if (lot.winner != -1)
        {
            players[lot.winner].buy(lot);
            std::cout << "Winner: Player " << players[lot.winner].getId() << " for price " << lot.price << "\n";
        }
        else
        {
            std::cout << "No bids this round.\n";
        }
    }
    finish();
}

void GameState::finish() const
{
	for (int i = 0; i < players.size(); i++)
	{
		std::cout << "Player " << players[i].getId() << " has " << players[i].getLots() << " lots.\n";
	}
}