#include "multiply.hpp"
#include "point.hpp"
#include "pow.hpp"
#include <iostream>

int main()
{
    std::cout << Pow<-2>::apply(2) << std::endl;
    std::cout << Multiply<2, 3>::value << std::endl;
    std::cout << Multiply<-2, 3>::value << std::endl;
    std::cout << Multiply<2, -3>::value << std::endl;
    std::cout << Multiply<1, 0>::value << std::endl;
    std::cout << Multiply<0, 2>::value << std::endl;
    std::cout << Multiply<0, 0>::value << std::endl;
    std::cout << MultiplyThree<2, 1, 3>::value << std::endl;
    std::cout << MultiplyThree<2, 1, -3>::value << std::endl;
    std::cout << MultiplyThree<-2, 1, -3>::value << std::endl;
    std::cout << MultiplyThree<-2, -1, -3>::value << std::endl;

    Point<double, 3> p1({1.0, 0.0, 3.0});
    Point<double, 3> p2({4.0, 4.0, 3.0});
    Point<float, 2> p3({4.0, 4.0});

    std::cout << "Coordinates of point 1: " << p1.to_string() << std::endl;
    std::cout << "Coordinates of point 2: " << p2.to_string() << std::endl;
    std::cout << "Coordinates of point 3: " << p3.to_string() << std::endl;
    std::cout << "Distance: " << Point<double, 3>::distance(p1, p2) << std::endl;
    Point<double, 3> p4 = Point<double, 3>::from_string_impl("(10.0, 0.0, 3.0)");
    std::cout << "Coordinates of point 4: " << p4.to_string() << std::endl;

    int A;
    int B;
    int C;
    std::cin >> A >> B >> C;
    std::cout << Pow_apply(A, B) << std::endl;
    std::cout << Multiply_apply(A, B, C) << std::endl;
    std::cout << Multiply_apply(A, B) << std::endl;
    return 0;
}