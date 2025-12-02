#ifndef GAME_HPP
#define GAME_HPP
#include <vector>
#include <iostream>
#include <thread>
#include <mutex>
#include "player.hpp"
#include "task.hpp"

extern std::queue<char> input_queue;
extern std::mutex input_mutex;
extern std::atomic<bool> input_running;

void input_thread_func();

Task bidder(Player& p, Lot& lot, std::mt19937& gen, bool& someone_bid);
Task human_bidder(Player& p, Lot& lot, bool& someone_bid);

void run_round(std::vector<Player>& players, int lot_id, std::mt19937& gen);

#endif