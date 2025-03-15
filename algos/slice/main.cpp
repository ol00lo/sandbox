#include "slice.hpp"

int main()
{
    std::tuple<int, int, char> t{7, 8, 'a'};

    auto t1 = make_tuple_slice<0, 2>(t);
    std::cout << " tuple<int, char>{7, a} == " << type_name<decltype(t1)>() << "{" << std::get<0>(t1) << ", "
              << std::get<1>(t1) << "}" << std::endl;

    std::tuple<int, int, char, bool> t2{7, 8, 'a', true};

    auto t3 = make_tuple_slice_range<1, 4>(t2);
    std::cout << " tuple<int, char, bool>{8, a, 1} == " << type_name<decltype(t3)>() << "{" << std::get<0>(t3) << ", "
              << std::get<1>(t3) << ", "<< std::get<2>(t3) << "}" << std::endl;

    auto t4 = make_tuple_slice_range<2, 4>(t2);
    std::cout << " tuple<char, bool>{a, 1} == " << type_name<decltype(t4)>() << "{" << std::get<0>(t4) << ", "
              << std::get<1>(t4) << "}" << std::endl;
}