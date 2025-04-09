#ifndef GENERATOR_H
#define GENERATOR_H
#include "tensor.hpp"
#include <random>
#include <vector>

namespace g
{
using tvec_t = std::vector<Tensor>;

struct IDataGenerator
{
public:
    IDataGenerator(int seed = 0) : rng_(seed) {};

    virtual tvec_t next_input() = 0;
    virtual tvec_t next_gt() = 0;

    virtual bool is_epoch_end() = 0;
    virtual void next_epoch(bool shuffle) = 0;

    virtual ~IDataGenerator() {};

protected:
    std::mt19937 rng_;
};

struct SimpleDataGenerator : public IDataGenerator
{
public:
    SimpleDataGenerator(const std::vector<tvec_t>& in, const std::vector<tvec_t>& out, int batch_size = 1, int seed = 0);

    tvec_t next_input() override;
    tvec_t next_gt() override;

    bool is_epoch_end() override;
    void next_epoch(bool shuffle) override;

private:
    std::vector<tvec_t> inputs_;
    std::vector<tvec_t> outputs_;
    size_t input_index_ = 0;
    size_t gt_index_ = 0;
    size_t epoch_ = 0;
    int batch_size_;
    std::uniform_int_distribution<std::size_t> distribution_;
};

} // namespace g
#endif