#include "is.hpp"

int main()
{
    auto seq0 = std::integer_sequence<int, 0, 5, 8>{};
    std::cout << seq0 << std::endl;
    auto seq1 = make_sequence<5, 10, 2>();
    auto seq2 = make_sequence<10, 1, -2>();
    std::cout << seq1 << std::endl; // <5, 7, 9>
    std::cout << seq2 << std::endl; // <10, 8, 6, 4, 2>

    std::cout << sequence_entry<1>(seq1) << std::endl; // 7
    // std::cout << sequence_entry<3>(seq1) << std::endl; // compile error

    std::cout << sequence_entry(seq1, 1) << std::endl; // 7
    // std::cout << sequence_entry(seq1, 3) << std::endl; // runtime error
}