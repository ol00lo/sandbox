#include "generator.hpp"
#include <random>

std::mt19937 rng(0);
using namespace g;
SimpleDataGenerator::SimpleDataGenerator(const std::vector<std::vector<Tensor>>& in,
                                         const std::vector<std::vector<Tensor>>& out)
    : inputs(in), outputs(out)
{
}
std::vector<Tensor> SimpleDataGenerator::next_input(int batch_size)
{
    if (batch_size != 1)
    {
        throw std::runtime_error("Not implemented");
    }
    return inputs[current_index];
}
std::vector<Tensor> SimpleDataGenerator::next_gt(int batch_size)
{
    if (batch_size != 1)
    {
        throw std::runtime_error("Not implemented");
    }
    return outputs[current_index++];
}
bool SimpleDataGenerator::is_epoch_end()
{
    return current_index >= inputs.size();
}
void SimpleDataGenerator::next_epoch(bool shuffle)
{
    current_index = 0;
    if (shuffle)
    {
        for (int i = 0; i < inputs.size(); i++)
        {
            std::uniform_int_distribution<std::size_t> distribution(0, inputs.size() - 1);
            std::size_t j = distribution(rng);

            std::swap(inputs[i], inputs[j]);
            std::swap(outputs[i], outputs[j]);
        }
    }
}