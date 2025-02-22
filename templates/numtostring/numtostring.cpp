#include "numtostring.hpp"
#include "long_scale_nums.hpp"
// #include "short_scale_nums.hpp"
#include <iostream>
#ifdef _WIN32
#include "Windows.h"
#endif

using namespace num;

int main()
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    setlocale(LC_ALL, "ru");

    std::cout << num::NumToString<2, 2, 3, 3, 4, 5, 6>::apply() << std::endl;
    std::cout << num::NumToString<1, 5, 6, 2, 9, 8, 2, 2, 3, 1, 4, 5, 6>::apply() << std::endl;
    std::cout << num::NumToString<1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0>::apply() << std::endl;
    std::cout << num::NumToString<1, 0, 0, 0, 0, 0, 0>::apply() << std::endl;
    std::cout << num::NumToString<0, 0, 0, 5>::apply() << std::endl;
    std::cout << num::Int<-453>::str() << std::endl;

    return 0;
}