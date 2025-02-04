#include "i_node.hpp"
#include <random>
using namespace g;

namespace
{
std::mt19937 rnd;
std::string build_random_string(int length)
{
    std::string res;
    static const char alphanum[] = "0123456789"
                                   "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                                   "abcdefghijklmnopqrstuvwxyz";

    std::uniform_int_distribution<> dist(0, sizeof(alphanum) - 1);
    for (int i = 0; i < length; ++i)
    {
        res += alphanum[dist(rnd)];
    }
    return res;
}

std::string set_nodename(std::string nodename, std::unordered_set<std::string>& existing_names)
{
    if (!nodename.empty() && existing_names.find(nodename) != existing_names.end())
    {
        throw std::runtime_error("Node name must be unique: " + nodename);
    }
    if (nodename.empty())
    {
        nodename = build_random_string(8);
    }
    int trycount = 0;
    while (existing_names.find(nodename) != existing_names.end())
    {
        trycount++;
        if (trycount > 10)
        {
            throw std::runtime_error("Failed to generate unique name.");
        }
        nodename = build_random_string(8);
    }
    return nodename;
}
} // namespace
INode::INode(std::string nodename) : _nodename(set_nodename(nodename, _existing_names))
{
    _existing_names.insert(_nodename);
}
INode::PNode INode::factory(std::string classname, std::string nodename)
{
    auto fnd = _registered_classes.find(classname);
    if (fnd == _registered_classes.end())
    {
        throw std::runtime_error("Invalid classname: " + classname);
    }
    return fnd->second(nodename);
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
        prev.push_back(a->nodename());
    }
    nlohmann::json res = {{"classname", classname()}, {"nodename", _nodename}, {"prev_nodes", prev}};
    serialize_spec(res);
    log().debug("Node {} of the {} class is serialized.", _nodename, classname());
    return res;
}
std::string INode::nodename() const
{
    return _nodename;
}
bool INode::register_class(std::string classname, NodeBuilder builder)
{
    _registered_classes[classname] = builder;
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
    for (auto& n : _next_nodes)
    {
        n->clear_forward_cache();
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

void g::set_dep(std::shared_ptr<INode> node, std::initializer_list<std::shared_ptr<INode>> prevs)
{
    for (auto& prev_node : prevs)
    {
        node->add_prev(prev_node);
        prev_node->add_next(node);
    }
}