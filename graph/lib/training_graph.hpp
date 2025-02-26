#ifndef TRAINING_GRAPH_H
#define TRAINING_GRAPH_H
#include "model.hpp"

namespace g
{
class TrainingGraph : public Model
{
public:
    TrainingGraph(Model target_model, std::string loss_type);
    void copy_to_target();
    std::vector<Tensor> get_gradients() const;

private:
    Model _target_model;
};
} // namespace g

#endif