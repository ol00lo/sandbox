#ifndef FUNCTIONAL_NODE_HPP
#define FUNCTIONAL_NODE_HPP

#include "i_node.hpp"
#include <map>

namespace g
{
class IFunctionalNode : public INode
{
public:
    IFunctionalNode(std::string classname, std::string nodename = "") : INode(classname, nodename), value_({0})
    {
    }
    using callback_t = std::function<void(IFunctionalNode*)>;

    void add_value_callback(callback_t cb);
    void add_gradient_callback(callback_t cb);
    Tensor value() override;
    Shape output_shape() const override;
    virtual ~IFunctionalNode() {};

protected:
    void clear_cache() override;

private:
    bool has_value_ = false;
    bool has_gradient_ = false;
    std::vector<Tensor> gradient_;
    Tensor value_;

    std::vector<callback_t> value_callbacks_;
    std::vector<callback_t> gradient_callbacks_;
    void log_cache()
    {
        log().debug(classname() + " cleaned");
    }
    void before_value_compute();
    void before_gradient_compute();
    virtual Tensor compute_value() = 0;
    std::map<const INode*, Tensor> derivative_cache_;

    Tensor notself_derivative(const INode* arg) override;
    Tensor compute_notself_derivative(const INode* arg);
    virtual std::vector<Tensor> gradient() = 0;
};

} // namespace g
#endif // !FUNCTIONAL_NODES_HPP
