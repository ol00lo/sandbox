#include "ratio.hpp"

int main()
{
    using t1 = Ratio<1, 4>;
    using t2 = Ratio<7, 6>;
    using t3 = ratio_add_t<t1, t2>;
    using t4 = ratio_mult_t<t1, t2>;

    std::cout << "17/12 == " << t3::str() << std::endl;
    std::cout << "7/24 == " << t4::str() << std::endl;
    std::cout << "0 == " << ratio_compare_v<Ratio<1, 3>, Ratio<2, 6>> << std::endl;
    std::cout << "-1 == " << ratio_compare_v<Ratio<-1, 3>, Ratio<2, 6>> << std::endl;
    std::cout << "1 == " << ratio_compare_v<Ratio<1, 3>, Ratio<-2, 6>><< std::endl;

    std::cout << Ratio<-1, 3>::str() << std::endl;
    std::cout << Ratio<1, -3>::str() << std::endl;
    std::cout << Ratio<-1, -3>::str() << std::endl;
    std::cout << ratio_simplify_t<Ratio<-9, -3>>::str() << std::endl;
}