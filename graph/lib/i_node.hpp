#ifndef I_NODE_HPP
#define I_NODE_HPP

#include <nlohmann/json.hpp>
#include "graph.hpp"
#include "tensor.hpp"
#include <functional>
#include <memory>
#include <vector>
#include <map>

#define REGISTER_INODE_CHILD(classname)                                                                                \
    static inline const bool __registered = INode::register_class(#classname, [](std::string nodename) {               \
        return std::static_pointer_cast<INode>(std::make_shared<classname>(nodename));                                 \
    })

namespace g
{
class INode
{
public:
    using ptr_t = std::shared_ptr<INode>;
    using node_builder_t = std::function<ptr_t(std::string nodename)>;
    explicit INode(std::string classname = "", std::string nodename = "");
    INode() = delete;
    INode(const INode&) = delete;
    INode& operator=(const INode&) = delete;
    static ptr_t factory(std::string classname, std::string nodename = "");
    void add_prev(std::shared_ptr<INode> a);
    void add_next(std::shared_ptr<INode> a);
    virtual Tensor value() = 0;
    std::string nodename() const;
    Tensor derivative(const INode* argument);
    Tensor derivative(std::shared_ptr<INode>);
    std::vector<std::shared_ptr<INode>> prev_nodes() const;
    std::vector<std::shared_ptr<INode>> next_nodes();
    void clear_prev();
    virtual std::string classname() const = 0;
    void set_dep(const nlohmann::json&, const std::unordered_map<std::string, std::shared_ptr<INode>>&);
    void rename();
    virtual void serialize_spec(nlohmann::json& js) const {};
    virtual void deserialize_spec(const nlohmann::json&) {};
    virtual Shape output_shape() const = 0;
    virtual ~INode() {};

protected:
    const std::string nodename_;
    static inline std::map<std::string, node_builder_t> registered_classes_;
    static bool register_class(std::string classname, node_builder_t builder);
    std::vector<std::shared_ptr<INode>> prev_nodes_;
    std::vector<std::shared_ptr<INode>> next_nodes_;
    virtual void clear_cache() {};
    void clear_backward_cache();
    void clear_forward_cache();

private:
    virtual Tensor notself_derivative(const INode* arg) = 0;
};
void set_dep(std::shared_ptr<INode>, std::initializer_list<std::shared_ptr<INode>>);
} // namespace g

namespace nlohmann
{
template <typename T>
struct adl_serializer<std::shared_ptr<T>, std::enable_if_t<std::is_base_of<g::INode, T>::value>>
{
    static void to_json(json& j, const g::INode::ptr_t& node)
    {
        std::vector<std::string> prev;
        for (auto& a : node->prev_nodes())
        {
            prev.push_back(a->nodename());
        }
        j = json{{"classname", node->classname()},
                 {"nodename", node->nodename()},
                 {"prev_nodes", prev},
                 {"value", node->value()}};
        node->serialize_spec(j);
    }

    static void from_json(const json& node_json, g::INode::ptr_t& node)
    {
        std::string classname = node_json.at("classname").get<std::string>();
        std::string nodename = node_json.at("nodename").get<std::string>();
        node = g::INode::factory(classname, nodename);
        node->deserialize_spec(node_json);
    }
};
} // namespace nlohmann
#endif
