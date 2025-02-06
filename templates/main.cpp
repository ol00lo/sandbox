#include <iostream>
#include "numtostring.hpp"
#ifdef _WIN32
#include "Windows.h"
#endif

int main()
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    setlocale(LC_ALL, "ru");
    std::cout << NumToString<1, 2, 3, 4>::apply() << std::endl;
    std::cout << NumToString<0, 9, 1, 2>::apply() << std::endl;
    std::cout << NumToString<0, 0, 4, 9>::apply() << std::endl;
    
    std::cout << NumToString<0, 0, 0, 8>::apply() << std::endl;
    std::cout << NumToString<0, 0, 0, 0>::apply() << std::endl;
    std::cout << NumToString<9, 9, 9, 9>::apply() << std::endl;

    std::cout << NumToString<0, 0, 0>::apply() << std::endl;
    std::cout << NumToString<1, 1, 1>::apply() << std::endl;
    std::cout << NumToString<1, 9>::apply() << std::endl;
    std::cout << NumToString<6>::apply() << std::endl;
 
    return 0;
}