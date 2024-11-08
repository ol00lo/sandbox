#include "graph.hpp"

class IFunctionalNode : public INode
{
public:
    using callback_t = std::function<void(IFunctionalNode*)>;

    void set_value(double val) override{};

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

class MultNode : public IFunctionalNode
{
protected:
    double compute_value() override;
};

class PlusNode : public IFunctionalNode
{
protected:
    double compute_value() override;
};

class MinusNode : public IFunctionalNode
{
protected:
    double compute_value() override;
};

class SqrNode : public IFunctionalNode
{
protected:
    double compute_value() override;
};

namespace op
{
std::shared_ptr<IFunctionalNode> mult(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
std::shared_ptr<IFunctionalNode> plus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
std::shared_ptr<IFunctionalNode> minus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
std::shared_ptr<IFunctionalNode> sqr(std::shared_ptr<INode> a);
} // namespace op