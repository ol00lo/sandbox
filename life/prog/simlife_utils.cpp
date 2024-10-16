#include "simlife_utils.hpp"
namespace
{
std::random_device rd;
std::mt19937 ran(rd());
} // namespace

std::vector<bool> random_input(int height, int width, int norganisms)
{
    std::vector<bool> res(width * height, false);
    std::uniform_int_distribution ind(0, width * height - 1);
    for (int i = 0; i < norganisms; i++)
    {
        int at = ind(ran);
        res[at] = true;
    }
    return res;
}
std::vector<bool> input_fromfile(std::string filename)
{
    std::vector<bool> res;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("unable to open file");
    }
    std::string line;
    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        bool value;
        while (ss >> value)
        {
            res.push_back(value);
        }
    }
    file.close();
    return res;
}
std::pair<int, int> dim_fromfile(std::string filename)
{
    int width = 0;
    int height = 0;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("unable to open file");
    }
    std::string line;

    if (std::getline(file, line))
    {
        std::stringstream ss(line);
        bool value;
        while (ss >> value)
        {
            width++;
        }
        height++;
    }
    while (std::getline(file, line))
    {
        height++;
    }
    if (width < 1 || height < 1)
    {
        throw std::runtime_error("incorrect input");
    }
    return {height, width};
}
std::pair<int, int> dim_fromline(std::string dimensions)
{
    size_t xPos = dimensions.find('x');
    int width = std::stoi(dimensions.substr(0, xPos));
    int height = std::stoi(dimensions.substr(xPos + 1));
    if (width < 1 || height < 1)
    {
        throw std::runtime_error("incorrect input");
    }
    return {height, width};
}