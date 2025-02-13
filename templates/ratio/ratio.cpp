#include "ratio.hpp"

int main()
{
    using t1 = Ratio<1, 4>;
    using t2 = Ratio<7, 6>;
    using t3 = Add<t1, t2>::type;
    using t4 = Mult<t1, t2>::type;

    std::cout << "17/12 == " << t3::str() << std::endl;
    std::cout << "7/24 == " << t4::str() << std::endl;
    std::cout << "0 == " << Compare<Ratio<1, 3>, Ratio<2, 6>>::value << std::endl;
    std::cout << "-1 == " << Compare<Ratio<-1, 3>, Ratio<2, 6>>::value << std::endl;
    std::cout << "1 == " << Compare<Ratio<1, 3>, Ratio<-2, 6>>::value << std::endl;

    std::cout << Ratio<-1, 3>::str() << std::endl;
    std::cout << Ratio<1, -3>::str() << std::endl;
    std::cout << Ratio<-1, -3>::str() << std::endl;
    std::cout << Simplify<Ratio<-9, -3>>::type::str() << std::endl;
}