#include "i_node.hpp"
using namespace g;

namespace
{
std::string build_random_string(int length)
{
    std::string res;
    static const char alphanum[] = "0123456789"
                                   "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                   "abcdefghijklmnopqrstuvwxyz";
    for (int i = 0; i < length; ++i)
    {
        res += alphanum[rand() % (sizeof(alphanum) - 1)];
    }
    return res;
}
} // namespace

INode::INode(std::string nodename)
{
    if (nodename.empty())
    {
        nodename = build_random_string(8);
    }
    if (_existing_names.find(nodename) != _existing_names.end())
    {
        throw std::runtime_error("Node name must be unique: " + nodename);
    }
    _layer_name = nodename;
    _existing_names.insert(nodename);
}
INode::PNode INode::factory(std::string classname, std::initializer_list<PNode> args)
{
    auto fnd = _registered_classes.find(classname);
    if (fnd == _registered_classes.end())
    {
        throw std::runtime_error("Invalid classname: " + classname);
    }
    return fnd->second(args);
}
void INode::add_prev(const std::shared_ptr<INode> a)
{
    _prev_nodes.push_back(a);
}
void INode::add_next(const std::shared_ptr<INode> a)
{
    _next_nodes.push_back(a);
}
nlohmann::json INode::serialize() const
{
    std::vector<std::string> prev;
    for (auto& a : _prev_nodes)
    {
        prev.push_back(a->get_layer_name());
    }
    return {{"classname", classname()}, {"nodename", _layer_name}, {"prev_nodes", prev}};
}
std::string INode::get_layer_name() const
{
    return _layer_name;
}
bool INode::register_class(std::string classname, NodeBuilder builder)
{
    _registered_classes[classname] = builder;
    std::cout << classname << " registered" << std::endl;
    return true;
}
void INode::clear_backward_cache()
{
    clear_cache();
    for (auto& n : _prev_nodes)
    {
        n->clear_backward_cache();
    }
}

void INode::clear_forward_cache()
{
    clear_cache();
    for (auto& n : _prev_nodes)
    {
        n->clear_backward_cache();
    }
}

double INode::get_derivative(const INode* argument)
{
    if (argument == this)
        return 1;
    else
        return notself_derivative(argument);
}

double INode::get_derivative(std::shared_ptr<INode> argument)
{
    return get_derivative(argument.get());
}

std::vector<std::shared_ptr<INode>> INode::get_prev()
{
    return _prev_nodes;
}