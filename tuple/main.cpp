#include "tuple_algos.hpp"

void my_print(int v)
{
    std::cout << "int " << v << std::endl;
};
void my_print(double v)
{
    std::cout << "double " << v << std::endl;
};

void my_print(bool v)
{
    std::cout << "bool " << v << std::endl;
};

int transform_entry(double x)
{
    return (int)x;
}
bool transform_entry(int x)
{
    return (bool)x;
}
int transform_entry(float x)
{
    return 2 * (int)x;
}

int main()
{
    std::cout << "for each:\n\n";
    using t_t = std::tuple<int, double, int>;
    t_t tt{1, 2.5, 3};
    auto set_zero = [](auto& v) { v = 0; };
    auto set_one = [](auto& v) { v = 1; };
    std::cout << "before: \nint " << std::get<0>(tt) << " double " << std::get<1>(tt) << " int " << std::get<2>(tt)
              << std::endl;
    for_each(tt, set_zero);
    std::cout << "\nafter set 0: \n";
    for_each(tt, [](auto v) { my_print(v); });
    for_each_fe(tt, set_one);
    std::cout << "\nafter set 1: \n";
    for_each_fe(tt, [](auto v) { my_print(v); });

    std::cout << "\nany of:\n\n";
    std::tuple<double, double, int> t1{1.0, -2.0, 6};
    bool r = any_of(t1, [](auto v) -> bool { return v < 0; });
    std::cout << std::boolalpha << "true == " << r << std::endl;
    r = any_of_fe(t1, [](auto v) -> bool { return v < 0; });
    std::cout << std::boolalpha << "true == " << r << std::endl;
    r = any_of(t1, [](auto v) -> bool { return v < -3; });
    std::cout << std::boolalpha << "false == " << r << std::endl;

    std::cout << "\ntransform:\n\n";
    auto op = [](const auto& v) { return transform_entry(v); };

    using in_t = std::tuple<float, double, int>;
    in_t h{1.f, 1.0, 1};
    std::cout << "type before transform: " << type_name<in_t>() << std::endl;
    using out_t = decltype(transform(h, op));
    std::cout << "after transform should be: std::tuple<int,int,bool>" << std::endl;
    std::cout << "result: " << type_name<out_t>() << std::endl;
    out_t t = transform(h, op);
    std::cout << "\nresult should be:\nint 2\nint 1\nbool 1\n\nresult: \n";
    for_each(t, [](auto v) { my_print(v); });
    std::cout << "\nresult fe:\n";
    out_t t2 = transform_fe(h, op);
    for_each(t2, [](auto v) { my_print(v); });
}