#ifndef TENSOR_INDEX_HPP
#define TENSOR_INDEX_HPP

#include <array>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

namespace g
{
using Arr4 = std::array<int, 4>;

class Shape : public Arr4
{
public:
    Shape(const Arr4& shape = {1, 1, 1, 1}) : Arr4{shape} {};
    Shape(int b, int i, int j, int c) : Arr4{b, i, j, c} {};
    Shape(int i, int j, int c) : Arr4{1, i, j, c} {};
    Shape(int j, int c) : Arr4{1, 1, j, c} {};
    Shape(int c) : Arr4{1, 1, 1, c} {};
    int n_indexes() const;
};

class Index : public Arr4
{
public:
    Index(int c) : Arr4{0, 0, 0, c} {};
    Index(int j, int c) : Arr4{0, 0, j, c} {};
    Index(int i, int j, int c) : Arr4{0, i, j, c} {};
    Index(int i, int j, int c, int b) : Arr4{i, j, c, b} {};
    Index(int linear, const Shape& shape);
    int to_linear(const Shape& shape) const;
};
} // namespace g

namespace nlohmann
{
template <>
struct adl_serializer<g::Shape>
{
    static void to_json(json& j, const g::Shape& s)
    {
        j = "(" + std::to_string(s[0]) + ", " + std::to_string(s[1]) + ", " + std::to_string(s[2]) + ", " +
            std::to_string(s[3]) + ")";
    }

    static void from_json(const json& j, g::Shape& s)
    {
        std::string str = j.get<std::string>();
        str.erase(0, 1);
        str.erase(str.size() - 1);
        std::stringstream ss(str);
        std::string item;
        int i = 0;
        while (std::getline(ss, item, ','))
        {
            s[i] = std::stoi(item);
            i++;
        }
    }
};
} // namespace nlohmann
#endif