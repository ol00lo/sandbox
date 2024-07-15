#include "timer.h"
#include <iostream>
#include <vector>
#include <functional>
#include <thread>
#include <future>
#include <mutex>

#define CHECK(x) if(!(x)) {errors(done, __LINE__);}
#define CHECK_NOTHROW(x) try{x;}catch(...){errors(done, __LINE__);}
#define CHECK_THROWS_AS(x, y) help=0; try{x;}catch(y){help++;}catch(...){help=0;} if(help!=1){errors(done, __LINE__);}
std::atomic_bool is_break = false;
std::atomic_int num_after = 0;

void errors(int& done, int line) {
	done++;
	std::cout << "error in " << line << std::endl;
}

class MyException : public std::runtime_error {
public:
	MyException(std::string msg) : std::runtime_error(msg) {}
};


class ThreadPool {
public:
	using task_t = std::function<double(int)>;
	ThreadPool(int n_threads, task_t task) : _task(task), _n_threads(n_threads) {}

	std::vector<double> run(const std::vector<int>& input) {
		is_ready = is_excp = false;
		std::vector<int> args = input;
		std::vector<double> res(args.size());
		std::vector<std::thread> threads;
		for (int i = 0; i < _n_threads; i++)
			threads.emplace_back([&] {worker(args, res); });
		for (auto& t : threads)
			t.join();
		if (excp) 
			std::rethrow_exception(excp);
		return res;
	}

private:
	void worker(std::vector<int>& args, std::vector<double>& res) {
		while (!is_ready) {
			if (is_excp) 
				break;
			int index; int arg;
			{
				std::lock_guard<std::mutex> lock(_mtx);
				if (args.size() > 0) {
					index = args.size() - 1;
					arg = args[index];
					args.pop_back();
				}
				else
					is_ready = true;
			}
			try {
				if (!is_ready)
					res[index] = _task(arg);
			}
			catch (std::exception& e) {
				excp = std::current_exception();
				is_excp = true;
			}
		}
	}
	task_t _task;
	int _n_threads;
	std::mutex _mtx;
	std::atomic_bool is_ready = false;
	std::atomic_bool is_excp = false;
	std::exception_ptr excp;
};

double heavy_function2(int x) {
	if (is_break) num_after++;
	if (x == 16) {
		std::this_thread::sleep_for(std::chrono::milliseconds(2));
		is_break = true;
		throw MyException("exception at x=16");
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(10));
	return (double)x;
}

double heavy_function1(int x) {
	size_t n = 400;
	double res = 0;
	for (size_t i = 0; i < n; i++)
		for (size_t j = 0; j < n; j++)
			res += log(x);
	return (double)x;
}

void f1() {
	int done = 0;
	std::vector<int> arguments;
	for (int i = 0; i < 1000; ++i) arguments.push_back(i);

	ThreadPool pool(16, heavy_function1);
	std::vector<double> result = pool.run(arguments);

	CHECK(result.size() == 1000);
	CHECK(result[0] == 0.0);
	CHECK(result[999] == 999.0);

	arguments.resize(4);
	result = pool.run(arguments);
	CHECK(result.size() == 4);


	if (done == 0) std::cout << "DONE" << std::endl;
	else std::cout << "Count of errors: " << done;
}

void f2() {
	int done = 0;
	int help;
	std::vector<int> arguments;
	for (int i = 0; i < 16; ++i) arguments.push_back(i);

	ThreadPool pool(4, heavy_function2);
	CHECK_NOTHROW(pool.run(arguments));
	CHECK_NOTHROW(pool.run({ 1,2,3,14 }));

	arguments.push_back(16);
	CHECK_THROWS_AS(pool.run(arguments), MyException);

	ThreadPool pool2(1, heavy_function2);
	arguments = { 1,2,3,16,4,5 };
	is_break = false;
	num_after = 0;
	CHECK_THROWS_AS(pool2.run(arguments), MyException);
	CHECK(num_after == 0);

	arguments = { 115,114,113,112,111,110,109,108,107,106,105,104,103,102,101,16,4,5 };
	is_break = false;
	num_after = 0;
	CHECK_THROWS_AS(pool.run(arguments), MyException);
	CHECK(num_after == 0);

	if (done == 0) std::cout << "DONE" << std::endl;
	else std::cout << "Count of errors: " << done;
}

int main() {
	f1();
	f2();
}