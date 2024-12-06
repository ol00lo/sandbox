#ifndef I_NODE_HPP
#define I_NODE_HPP

#include <nlohmann/json.hpp>
#include "graph.hpp"
#include <functional>
#include <memory>
#include <vector>
#include <map>
#include <unordered_set>

#define REGISTER_INODE_CHILD(classname)                                                                                \
    static inline const bool __registered = INode::register_class(                                                     \
        #classname, [](std::initializer_list<std::shared_ptr<INode>> args) {                                           \
        return std::static_pointer_cast<INode>(                                                                        \
                std::make_shared<classname>(args));                                                                    \
    })

namespace g
{
class INode
{
public:
    using PNode = std::shared_ptr<INode>;
    using NodeBuilder = std::function<PNode(std::initializer_list<PNode> args)>;
    INode(std::string nodename = "");
    static PNode factory(std::string classname, std::initializer_list<PNode> args);
    virtual void set_value(double val) {};
    void add_prev(std::shared_ptr<INode> a);
    void add_next(std::shared_ptr<INode> a);
    virtual double get_value() = 0;
    std::string get_layer_name() const;
    double get_derivative(const INode* argument);
    double get_derivative(std::shared_ptr<INode>);
    std::vector<std::shared_ptr<INode>> get_prev();
    virtual std::string classname() const = 0;
    virtual nlohmann::json serialize() const;
    virtual ~INode()
    {
        _existing_names.erase(_layer_name);
    };
    static inline std::unordered_set<std::string> _existing_names;

protected:
    std::string _layer_name;
    static inline std::map<std::string, NodeBuilder> _registered_classes;
    static bool register_class(std::string classname, NodeBuilder builder);
    std::vector<std::shared_ptr<INode>> _prev_nodes;
    std::vector<std::shared_ptr<INode>> _next_nodes;
    virtual void clear_cache() {};
    void clear_backward_cache();
    void clear_forward_cache();

private:
    virtual double notself_derivative(const INode* arg) = 0;
};
} // namespace g
#endif
