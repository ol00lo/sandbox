#include "game.hpp"

std::queue<char> input_queue;
std::atomic<bool> input_running = true;
std::mutex input_mutex;

void input_thread_func() {
    while (input_running) {
        char c;
        std::cin.get(c);
        if (c == '\n')
            continue;
        std::lock_guard<std::mutex> lock(input_mutex);
        input_queue.push(c);
    }
}

Task bidder(Player& p, Lot& lot, std::mt19937& gen, bool& someone_bid) {
    std::uniform_real_distribution<double> prob(0.0, 1.0);

    while (true) {
        if (p.getCoins() <= lot.price) co_return;

        if (prob(gen) < p.getProbability()) {
            int new_price = p.bid(lot, gen);
            if (new_price != -1) {
                lot.price = new_price;
                lot.winner = p.getId();
                someone_bid = true;
                p.updateBitTime();

                co_await SleepFor(std::chrono::seconds(5));
                continue;
            }
            else co_return;
        }
        else co_await SleepFor(std::chrono::milliseconds(200));

        co_await YieldOnce{};
    }
}

Task human_bidder(Player& p, Lot& lot, bool& someone_bid) {
    while (true) {
        if (!p.isHuman())
            co_return;
        if (std::chrono::steady_clock::now() - p.last_bid_time < std::chrono::seconds(5)) {
            co_await SleepFor{std::chrono::milliseconds(100)};
            continue;
        }
        char c = 0;
        {
            std::lock_guard<std::mutex> lock(input_mutex);
            if (!input_queue.empty()) {
                c = input_queue.front();
                input_queue.pop();
            }
        }

        if (c >= '1' && c <= '9') {
            int increase = c - '0';
            int new_price = lot.price + increase;
            if (new_price <= p.getCoins()) {
                lot.price = new_price;
                lot.winner = p.getId();
                someone_bid = true;
                p.updateBitTime();
            }
        }
        co_await SleepFor{std::chrono::milliseconds(50)};
    }
}

Task status_printer(const std::vector<Player>& players, const Lot& current_lot) {
    while (true) {
        std::cout << "\x1B[2J\x1B[H";

        std::cout << "=== Current Lot ===\n";
        std::cout << "Lot ID: " << current_lot.id << " | Current Price: " << current_lot.price;
        if (current_lot.winner != -1) std::cout << " | Last Bidder: Player " << current_lot.winner;
        std::cout << "\n\n";

        std::cout << std::left << std::setw(8) << "Player" << std::setw(8) << "Coins" << std::setw(8) << "Lots"
                  << std::setw(12) << "Last Bid"
                  << "\n";
        std::cout << std::string(36, '-') << "\n";

        auto now = std::chrono::steady_clock::now();
        for (const auto& p : players) {
            auto last_bid_sec = p.last_bid_time == std::chrono::steady_clock::time_point{}
                                    ? -1
                                    : duration_cast<std::chrono::seconds>(now - p.last_bid_time).count();

            bool is_last = current_lot.winner == p.getId();
            if (is_last) std::cout << "\x1B[32m";

            std::cout << std::left << std::setw(8) << p.getId() << std::setw(8) << p.getCoins() << std::setw(8)
                      << p.getLots() << std::setw(12)
                      << (last_bid_sec >= 0 ? std::to_string(last_bid_sec) + "s" : "never") << "\n";

            if (is_last) std::cout << "\x1B[0m";
        }

        std::cout << std::flush;
        co_await SleepFor{std::chrono::milliseconds(900)};
    }
}

void run_round(std::vector<Player>& players, int lot_id, std::mt19937& gen) {
    Lot lot;
    lot.id = lot_id;

    bool someone_bid = false;

    for (auto& p : players) {
        if (p.isHuman()) human_bidder(p, lot, someone_bid);
        else bidder(p, lot, gen, someone_bid);
    }

    status_printer(players, lot);
    auto silence_start = std::chrono::steady_clock::now();

    const std::chrono::milliseconds tick_sleep(50);

    while (true) {
        someone_bid = false;

        Scheduler::instance().run_once();

        if (someone_bid) silence_start = std::chrono::steady_clock::now();

        if (std::chrono::steady_clock::now() - silence_start >= std::chrono::seconds(4)) break;

        std::this_thread::sleep_for(tick_sleep);
    }

    if (lot.winner != -1) players[lot.winner].buy(lot);
}