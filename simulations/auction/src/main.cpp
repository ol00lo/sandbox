#include "game.hpp"

int main()
{
    std::mt19937 gen(std::random_device{}());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    int n_players = 10;
    int n_lots = 15;
    int start_coins = 10;

    std::vector<Player> players;
    for (int i = 0; i < n_players; i++)
        players.emplace_back(start_coins, gen);

    players.emplace_back(start_coins, gen, true);

    std::thread input_thread(input_thread_func);

    for (int r = 0; r < n_lots; r++)
        run_round(players, r, gen);

    input_running = false;
    input_thread.join();
}
