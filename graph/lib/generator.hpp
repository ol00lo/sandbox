#ifndef GENERATOR_H
#define GENERATOR_H
#include <vector>
#include "tensor.hpp"
#include <random>

namespace g
{
using tvec_t = std::vector<Tensor>;

struct IDataGenerator
{
public:
    IDataGenerator(int seed = 0): _rng(seed){}
    virtual tvec_t next_input() = 0;
    virtual tvec_t next_gt() = 0;
    virtual bool is_epoch_end() = 0;
    virtual void next_epoch(bool shuffle) = 0;
	virtual ~IDataGenerator() {}

protected:
    std::mt19937 _rng;
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
    std::vector<tvec_t> _inputs;
    std::vector<tvec_t> _outputs;
    size_t _input_index = 0;
    size_t _gt_index = 0;
    int _batch_size;
    std::uniform_int_distribution<std::size_t> _distribution;
};

} // namespace g
#endif