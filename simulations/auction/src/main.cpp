#include <iostream>
#include "game.hpp"

int main() {
    const int nrounds = 10;
    const int nplayers = 3;
    const int start_money = 10;

    std::vector<Player> players;
    players.reserve(nplayers);
    for (int i = 0; i < nplayers; ++i)
        players.emplace_back(start_money);

	GameState game(nrounds, players);
	game.run();
    return 0;
}
