#include "generator.hpp"

using namespace g;
SimpleDataGenerator::SimpleDataGenerator(const std::vector<tvec_t>& in, const std::vector<tvec_t>& out, int batch_size, int seed)
    : IDataGenerator(seed), _inputs(in), _outputs(out), _batch_size(batch_size), _distribution(0, _inputs.size() - 1)
{
    if (in[0][0].get_shape()[0] != 1 || out[0][0].get_shape()[0] != 1)
    {
        throw std::runtime_error("Expected batch size of 1");
    }
}
tvec_t SimpleDataGenerator::next_input()
{
    if (_batch_size != 1)
    {
        _THROW_NOT_IMP_
    }
    return _inputs[_input_index++];
}
tvec_t SimpleDataGenerator::next_gt()
{
    if (_batch_size != 1)
    {
        _THROW_NOT_IMP_
    }
    return _outputs[_gt_index++];
}
bool SimpleDataGenerator::is_epoch_end()
{
    return _input_index >= _inputs.size() || _gt_index >= _outputs.size();
}
void SimpleDataGenerator::next_epoch(bool shuffle)
{
    _input_index = 0;
    _gt_index = 0;
    if (shuffle)
    {
        for (int i = 0; i < _inputs.size(); i++)
        {
            std::size_t j = _distribution(_rng);
            std::swap(_inputs[i], _inputs[j]);
            std::swap(_outputs[i], _outputs[j]);
        }
    }
}