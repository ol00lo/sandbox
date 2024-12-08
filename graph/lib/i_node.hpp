#ifndef I_NODE_HPP
#define I_NODE_HPP

#include "graph.hpp"
#include <functional>
#include <memory>
#include <vector>

namespace g
{
class INode
{
public:
    virtual double get_value() = 0;
    void add_prev(std::shared_ptr<INode> a);
    void add_next(std::shared_ptr<INode> a);
    virtual ~INode(){};
    double get_derivative(const INode* argument);
    double get_derivative(std::shared_ptr<INode>);

protected:
    std::vector<std::shared_ptr<INode>> _prev_nodes;
    std::vector<std::shared_ptr<INode>> _next_nodes;

    virtual void clear_cache(){};
    void clear_forward_cache();
    void clear_backward_cache();

    virtual std::string classname() const = 0;
private:
    virtual double notself_derivative(const INode* arg) = 0;

};
} // namespace g
#endif
