#ifndef FUNCTIONAL_NODES_HPP
#define FUNCTIONAL_NODES_HPP

#include "i_node.hpp"

namespace g
{
class IFunctionalNode : public INode
{
public:
    using callback_t = std::function<void(IFunctionalNode*)>;

    void add_value_callback(callback_t cb);

    double get_value() override;

protected:
    void clear_cache() override;

private:
    bool _has_value = false;
    double _value = 0;
    std::vector<callback_t> _value_callbacks;

    void before_value_compute();
    virtual double compute_value() = 0;
};
} // namespace g
#endif // !FUNCTIONAL_NODES_HPP
