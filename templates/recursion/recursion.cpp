#include "multiply.hpp"
#include "point.hpp"
#include "pow.hpp"
#include <iostream>

int main()
{
    std::cout << "0.25 == " << Pow<-2>::apply(2) << std::endl;
    std::cout << "6 == " << Multiply<2, 3>::value << std::endl;
    std::cout << "-6 == " << Multiply<-2, 3>::value << std::endl;
    std::cout << "-6 == " << Multiply<2, -3>::value << std::endl;
    std::cout << "0 == " << Multiply<1, 0>::value << std::endl;
    std::cout << "0 == " << Multiply<0, 2>::value << std::endl;
    std::cout << "0 == " << Multiply<0, 0>::value << std::endl;
    std::cout << "6 == " << MultiplyThree<2, 1, 3>::value << std::endl;
    std::cout << "-6 == " << MultiplyThree<2, 1, -3>::value << std::endl;
    std::cout << "6 == " << MultiplyThree<-2, 1, -3>::value << std::endl;
    std::cout << "-6 == " << MultiplyThree<-2, -1, -3>::value << std::endl;

    Point<double, 3> p1({1.0, 0.0, 3.0});
    Point<double, 3> p2({4.0, 4.0, 3.0});
    Point<float, 2> p3({4.0, 4.0});

    std::cout << "(1.0, 0.0, 3.0): " << p1.to_string() << std::endl;
    std::cout << "(4.0, 4.0, 3.0): " << p2.to_string() << std::endl;
    std::cout << "(4.0, 4.0): " << p3.to_string() << std::endl;
    std::cout << "5 == " << p1.distance(p2) << std::endl;
    Point<double, 3> p4 = Point<double, 3>::from_string_impl("(10.0, 0.0, 3.0)");
    std::cout << "Coordinates of point 4: " << p4.to_string() << std::endl;

    int A;
    int B;
    int C;
    std::cin >> A >> B >> C;
    std::cout << "B^A= " << Pow_apply(A, B) << std::endl;
    std::cout << "B^A= " << pow_apply(A, B) << std::endl;
    std::cout << "A*B*C= " << Multiply_apply(A, B, C) << std::endl;
    std::cout << "A*B= "<< Multiply_apply(A, B) << std::endl;
    return 0;
}