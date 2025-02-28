#ifndef GENERATOR_H
#define GENERATOR_H
#include <vector>
#include "tensor.hpp"

namespace g
{
struct IDataGenerator
{
public:
    virtual std::vector<Tensor> next_input(int batch_size) = 0;
    virtual std::vector<Tensor> next_gt(int batch_size) = 0;
    virtual bool is_epoch_end() = 0;
    virtual void next_epoch(bool shuffle) = 0;
};

struct SimpleDataGenerator : public IDataGenerator
{
public:
    SimpleDataGenerator(const std::vector<std::vector<Tensor>>& in, const std::vector<std::vector<Tensor>>& out);
    std::vector<Tensor> next_input(int batch_size = 1) override;
    std::vector<Tensor> next_gt(int batch_size = 1) override;
    bool is_epoch_end() override;
    void next_epoch(bool shuffle) override;

private:
    std::vector<std::vector<Tensor>> inputs;
    std::vector<std::vector<Tensor>> outputs;
    size_t current_index = 0;
};

} // namespace g
#endif