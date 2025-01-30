#ifndef FUNCTIONAL_NODE_HPP
#define FUNCTIONAL_NODE_HPP

#include "i_node.hpp"
#include <map>

namespace g
{
class IFunctionalNode : public INode
{
public:
    IFunctionalNode(std::string nodename = "") : INode(nodename), _value({0})
    {
    }
    using callback_t = std::function<void(IFunctionalNode*)>;

    void add_value_callback(callback_t cb);
    void add_gradient_callback(callback_t cb);
    Tensor get_value() override;
    Shape get_shape() const override;
    virtual ~IFunctionalNode() {};

protected:
    void clear_cache() override;

private:
    bool _has_value = false;
    bool _has_gradient = false;
    std::vector<Tensor> _gradient;
    Tensor _value;

    std::vector<callback_t> _value_callbacks;
    std::vector<callback_t> _gradient_callbacks;
    void log_cache()
    {
        log().debug(classname() + " cleaned");
    }
    void before_value_compute();
    void before_gradient_compute();
    virtual Tensor compute_value() = 0;
    std::map<const INode*, Tensor> _derivative_cache;

    Tensor notself_derivative(const INode* arg) override;
    Tensor compute_notself_derivative(const INode* arg);
    virtual std::vector<Tensor> get_gradient() = 0;
};

} // namespace g
#endif // !FUNCTIONAL_NODES_HPP
