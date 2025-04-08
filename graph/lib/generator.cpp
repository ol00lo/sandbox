#include "generator.hpp"

using namespace g;
SimpleDataGenerator::SimpleDataGenerator(const std::vector<tvec_t>& in, const std::vector<tvec_t>& out, int batch_size, int seed)
    : IDataGenerator(seed), inputs_(in), outputs_(out), batch_size_(batch_size), distribution_(0, inputs_.size() - 1)
{
    if (in[0][0].shape()[0] != 1 || out[0][0].shape()[0] != 1)
    {
        throw std::runtime_error("Expected batch size of 1");
    }
}
tvec_t SimpleDataGenerator::next_input()
{
    if (batch_size_ != 1)
    {
        _THROW_NOT_IMP_
    }
    return inputs_[input_index_++];
}
tvec_t SimpleDataGenerator::next_gt()
{
    if (batch_size_ != 1)
    {
        _THROW_NOT_IMP_
    }
    return outputs_[gt_index_++];
}
bool SimpleDataGenerator::is_epoch_end()
{
    return input_index_ >= inputs_.size() || gt_index_ >= outputs_.size();
}
void SimpleDataGenerator::next_epoch(bool shuffle)
{
    log().info("Epoch {}", epoch_++);
    input_index_ = 0;
    gt_index_ = 0;
    if (shuffle)
    {
        for (int i = 0; i < inputs_.size(); i++)
        {
            std::size_t j = distribution_(rng_);
            std::swap(inputs_[i], inputs_[j]);
            std::swap(outputs_[i], outputs_[j]);
        }
    }
}