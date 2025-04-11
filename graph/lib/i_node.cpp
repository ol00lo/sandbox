#include "i_node.hpp"
#include <random>
using namespace g;

namespace
{
std::mt19937 rnd;
std::string build_random_string(int length, std::string classname)
{
    std::string res = classname + "_";
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

std::string set_nodename(std::string classname, std::string nodename)
{
    return (nodename.empty()) ? build_random_string(8, classname) : nodename;
}
} // namespace

INode::INode(std::string classname, std::string nodename) : nodename_(set_nodename(classname, nodename)){};
INode::ptr_t INode::factory(std::string classname, std::string nodename)
{
    auto fnd = registered_classes_.find(classname);
    if (fnd == registered_classes_.end())
    {
        throw std::runtime_error("Invalid classname: " + classname);
    }
    return fnd->second(nodename);
}
void INode::add_prev(const std::shared_ptr<INode> a)
{
    prev_nodes_.push_back(a);
}
void INode::add_next(const std::shared_ptr<INode> a)
{
    next_nodes_.push_back(a);
}

void INode::set_dep(const nlohmann::json& node_json,
                    const std::unordered_map<std::string, std::shared_ptr<INode>>& all_nodes)
{
    if (!node_json.at("prev_nodes").empty())
    {
        for (const auto& prev_node_name : node_json.at("prev_nodes"))
        {
            auto prev_node = all_nodes.find(prev_node_name.get<std::string>());
            add_prev(prev_node->second);
            auto this_node = all_nodes.find(nodename_);
            prev_node->second->add_next(this_node->second);
        }
    }

    log().debug("Set dependencies for node {} of the {} class", nodename_, classname());
}
std::string INode::nodename() const
{
    return nodename_;
}
bool INode::register_class(std::string classname, node_builder_t builder)
{
    registered_classes_[classname] = builder;
    return true;
}
void INode::clear_backward_cache()
{
    clear_cache();
    for (auto& n : prev_nodes_)
    {
        n->clear_backward_cache();
    }
}

void INode::clear_forward_cache()
{
    clear_cache();
    for (auto& n : next_nodes_)
    {
        n->clear_forward_cache();
    }
}

Tensor INode::derivative(const INode* argument)
{
    if (argument->nodename() == this->nodename())
    {
        return Tensor(argument->output_shape(), 1.0);
    }
    else
        return notself_derivative(argument);
}

Tensor INode::derivative(std::shared_ptr<INode> argument)
{
    return derivative(argument.get());
}

std::vector<std::shared_ptr<INode>> INode::prev_nodes() const
{
    return prev_nodes_;
}

std::vector<std::shared_ptr<INode>> INode::next_nodes()
{
    return next_nodes_;
}
void INode::clear_prev()
{
    prev_nodes_.clear();
}

void g::set_dep(std::shared_ptr<INode> node, std::initializer_list<std::shared_ptr<INode>> prevs)
{
    for (auto& prev_node : prevs)
    {
        node->add_prev(prev_node);
        prev_node->add_next(node);
    }
}