#ifndef TRAINING_GRAPH_H
#define TRAINING_GRAPH_H
#include "model.hpp"

namespace g
{
enum struct LossType
{
    MSE
};
class TrainingGraph : public Model
{
public:
    TrainingGraph(const Model& target_model, LossType loss_type);
    void copy_to_target();
    std::vector<Tensor> get_gradients() const;

private:
    Model target_model_;
};
} // namespace g

#endif