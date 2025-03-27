#include "wint.hpp"
#include <iomanip>

void print()
{
    WInt<32, false> i(4294967295);
    std::cout << "decimal: " << i << std::endl;
    std::cout << "binary: " << i.print_binary() << std::endl;
    WInt<32> j(-2'147'483'648);
    std::cout << "decimal: " << j << std::endl;
    std::cout << "binary: " << j.print_binary() << std::endl;

    WInt<16, false> k(65535);
    std::cout << "decimal: " << k << std::endl;
    std::cout << "binary: " << k.print_binary() << std::endl;

}

void arithmetic()
{
    WInt<8, false> x(3);
    WInt<8, false> y(5);

    x += y;
    std::cout << "8 " << "  ==  " << x << std::endl;
    x -= y;
    std::cout << "3 " << "  ==  " << x << std::endl;
    x *= y;
    std::cout << "15" << "  == " << x << std::endl;
    x /= y;
    std::cout << "3 " << "  ==  " << x << std::endl;

    WInt<8> l(3);
    WInt<8> k(-3);
    std::cout << std::setw(5) << "l " << "  ==  " << l << std::endl;
    std::cout << std::setw(5) << "k " << "  ==  " << k << std::endl;
    l += k;
    std::cout << std::setw(5) << "l+=k" << "  ==  " << l << std::endl;
    k += k;
    std::cout << std::setw(5) << "k+=k" << "  ==  " << k << std::endl;
    l *= k;
    std::cout << std::setw(5) << "l*=k" << "  ==  " << l << std::endl;
    k *= k;
    std::cout << std::setw(5) << "k*=k" << "  ==  " << k << std::endl;
    WInt<8> m = -k;
    std::cout << std::setw(5) << "k" << "  ==  " << k << std::endl;
    std::cout << std::setw(5) << "-k" << "  ==  " << m << std::endl;
}

void inequalities()
{
    WInt<8, false> x(3);
    WInt<8, false> y(5);
    WInt<8, false> z(5);

    std::cout << std::boolalpha;
    std::cout << std::setw(5) << "true" << " ==  " << (x < y) << std::endl;
    std::cout << std::setw(5) << "true" << " ==  " << (y > x) << std::endl;
    std::cout << std::setw(5) << "false" << " ==  " << (y > z) << std::endl;
    std::cout << std::setw(5) << "false" << " ==  " << (y < z) << std::endl;
    std::cout << std::setw(5) << "true" << " ==  " << (y <= z) << std::endl;
    std::cout << std::setw(5) << "true" << " ==  " << (y == z) << std::endl;
    std::cout << std::setw(5) << "true" << " ==  " << (y != x) << std::endl;

    WInt<8> l(3);
    WInt<8> k(-3);
    WInt<8> m(0);
    std::cout << std::setw(5) << "true" << " ==  " << (k < l) << std::endl;
    std::cout << std::setw(5) << "true" << " ==  " << (k < m) << std::endl;
    std::cout << std::setw(5) << "true" << " ==  " << (m < l) << std::endl;
    std::cout << std::setw(5) << "true" << " ==  " << (m <= l) << std::endl;
    k *= m;
    std::cout << std::setw(5) << "true" << " ==  " << (m == k) << std::endl;
    std::cout << std::setw(5) << "true" << " ==  " << (m <= k) << std::endl;
    std::cout << std::noboolalpha;
}

void cast()
{
    WInt<16> a(270);
    WInt<12> b(a);
    WInt<32> c(b);
    WInt<32> d(a);
    std::cout<< std::boolalpha;
    std::cout << std::setw(5) << "false" << " ==  " << (c != d) << std::endl;
    std::cout << std::noboolalpha;

    WInt<16, false> e(a);
    WInt<16, false> f(b);

    std::cout << a.print_binary() << " - 16, true\n" 
              << e.print_binary() << " - 16, false\n";
    std::cout << "      " << b.print_binary() << " -  8, true\n" << f.print_binary() << " - 16, false\n";

    WInt<8, false> g(255);
    std::cout << "  g  = " << g << " = <8, false> " << g.print_binary() << std::endl;
    WInt<8, true> h(g);
    std::cout << "h(g) = " << h << " = <8, true > " << h.print_binary() << std::endl;
}

void big_number()
{
    std::bitset<67> b;
    b.set(66);
    WInt<67, false> a(b);
    std::cout << "2^66= " << a << std::endl;
    std::cout << "      73,786,976,294,838,206,464" << std::endl;
}

int main()
{
    std::cout << "---------- PRINT TEST ----------\n";
    print();

    std::cout << "\n---------- ARITHMETIC TEST ----------\n";
    arithmetic();

    std::cout << "\n---------- INEQUALITIES TEST ----------\n";
    inequalities();

    std::cout << "\n---------- CAST TEST ----------\n";
    cast();

    std::cout << "\n---------- BIG NUMBER TEST ----------\n";
    big_number();

}