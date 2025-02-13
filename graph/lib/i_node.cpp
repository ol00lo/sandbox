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
INode::ptr_t INode::factory(std::string classname, std::string nodename)
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
void INode::set_dep(const nlohmann::json& node_json,
                        const std::unordered_map<std::string, std::shared_ptr<INode>>& all_nodes, std::string copy_word)
{
    if (!node_json.at("prev_nodes").empty())
    {
        for (const auto& prev_node_name : node_json.at("prev_nodes"))
        {
            auto prev_node = all_nodes.find(prev_node_name.get<std::string>() + copy_word);
            add_prev(prev_node->second);
            auto this_node = all_nodes.find(_nodename);
            prev_node->second->add_next(this_node->second);
        }
    }

    log().debug("Set dependencies for node {} of the {} class", _nodename, classname());
}
std::string INode::nodename() const
{
    return _nodename;
}
bool INode::register_class(std::string classname, node_builder_t builder)
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

Tensor INode::get_derivative(const INode* argument)
{
    if (argument == this)
    {
        return Tensor(argument->output_shape(), 1.0);
    }
    else
        return notself_derivative(argument);
}

Tensor INode::get_derivative(std::shared_ptr<INode> argument)
{
    return get_derivative(argument.get());
}

std::vector<std::shared_ptr<INode>> INode::get_prev() const
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