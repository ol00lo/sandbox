#include "optimizer.hpp"

using namespace g;

void SGDOptimizer::set_param_nodes(const std::vector<std::shared_ptr<DataNode>>& params)
{
    _params = params;
}

void SGDOptimizer::apply(const std::vector<Tensor>& gradients)
{
    for (size_t i = 0; i < _params.size(); i++)
    {

        Tensor delt = g::scalar_mult(_learning_rate, gradients[i]);
        Tensor old_value = _params[i]->get_value();
        Tensor res = g::sub(old_value, delt);
        _params[i]->set_value(res);
    }
}

void OptimizationWorker::set_optimizer(std::shared_ptr<IOptimizer> optimizer)
{
    _optimizer = optimizer;
}

double OptimizationWorker::train(std::vector<Tensor> inputs, std::vector<Tensor> gt, int batch_size)
{
    if (batch_size != 1)
    {
        throw std::runtime_error("Not implemented");
    }
    else if (inputs.size() != gt.size())
    {
        throw std::runtime_error("Inputs and ground truth must have the same size.");
    }
    _optimizer->set_param_nodes(_graph.get_param_nodes());
    double res = 0;
    int n_inp = inputs.size() / batch_size;
    for (int i_batch = 0; i_batch < batch_size; i_batch++)
    {
        std::vector<Tensor> local_inputs;
        for (int i = 0; i < n_inp; i++)
        {
            int ind = i_batch * n_inp + i;
            local_inputs.push_back(inputs[ind]);
            local_inputs.push_back(gt[ind]);
        }
        auto out = _graph.compute(local_inputs);
        double cur_res = 0;
        for (const auto& x : out)
        {
            if (x.get_shape() != Arr4{1, 1, 1, 1})
            {
                throw std::runtime_error("Not implemented");
            }
            cur_res += x[0];
        }
        auto grads = _graph.get_gradients();
        _optimizer->apply(grads);
        res += cur_res / double(out.size());
    }
    return res / double(batch_size);
}

double OptimizationWorker::validate(std::vector<Tensor> inputs, std::vector<Tensor> gt, int batch_size)
{
    if (batch_size != 1)
    {
        throw std::runtime_error("Not implemented");
    }
    else if (inputs.size() != gt.size())
    {
        throw std::runtime_error("Inputs and ground truth must have the same size.");
    }
    _optimizer->set_param_nodes(_graph.get_param_nodes());
    double res = 0;
    int n_inp = inputs.size() / batch_size;
    for (int i_batch = 0; i_batch < batch_size; i_batch++)
    {
        std::vector<Tensor> local_inputs;
        for (int i = 0; i < n_inp; i++)
        {
            int ind = i_batch * n_inp + i;
            local_inputs.push_back(inputs[ind]);
            local_inputs.push_back(gt[ind]);
        }
        auto out = _graph.compute(local_inputs);
        double cur_res = 0;
        for (const auto& x : out)
        {
            if (x.get_shape() != Arr4{1, 1, 1, 1})
            {
                throw std::runtime_error("Not implemented");
            }
            cur_res += x[0];
        }
        res += cur_res / double(out.size());
    }
    return res / double(batch_size);
}

void OptimizationWorker::commit()
{
    _graph.copy_to_target();
}