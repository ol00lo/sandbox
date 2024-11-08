#ifndef GRAPH_HPP
#define GRAPH_HPP

#include <functional>
#include <memory>
#include <vector>

class INode
{
public:
    virtual double get_value() = 0;
    virtual void set_value(double val) = 0;
    void add_prev(const std::shared_ptr<INode>& a);
    void add_next(const std::shared_ptr<INode>& a);

protected:
    std::vector<std::shared_ptr<INode>> _prev_nodes;
    std::vector<std::shared_ptr<INode>> _next_nodes;

    virtual void clear_cache(){};
    void clear_forward_cache();
    void clear_backward_cache();
};

class InputNode : public INode
{
public:
    double get_value() override;
    void set_value(double val) override;

private:
    double _value = 0;
};


#endif // !GRAPH_HPP