#ifndef FUNCTIONAL_NODES_HPP
#define FUNCTIONAL_NODES_HPP

#include "i_node.hpp"
#include <map>

namespace g
{
class IFunctionalNode : public INode
{
public:
    using callback_t = std::function<void(IFunctionalNode*)>;

    void add_value_callback(callback_t cb);
    double get_value() override;
    virtual ~IFunctionalNode() {};

protected:
    void clear_cache() override;

private:
    bool _has_value = false;
    double _value = 0;
    std::vector<callback_t> _value_callbacks;
    virtual void log_cache() = 0;
    void before_value_compute();
    virtual double compute_value() = 0;
    std::map<const INode*, double> _derivative_cache;

    double notself_derivative(const INode* arg) override;
    double compute_notself_derivative(const INode* arg);
    virtual std::vector<double> gradient() = 0;
};
void add_dependencies(std::shared_ptr<IFunctionalNode> node, std::initializer_list<std::shared_ptr<INode>> prevs);

} // namespace g
#endif // !FUNCTIONAL_NODES_HPP
