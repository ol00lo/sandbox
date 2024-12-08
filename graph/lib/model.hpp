#include "i_node.hpp"
#include "input_node.hpp"

namespace g
{
class Model
{
public:
    Model(std::vector<std::shared_ptr<INode>> inputs, std::vector<std::shared_ptr<INode>> outputs);
    void save(const std::string& filename);
    nlohmann::json serialize() const;
    static Model load(const std::string& filename);
    static Model deserialize(nlohmann::json);
    std::vector<double> compute(const std::vector<double>& input_values);

private:
    std::vector<std::shared_ptr<INode>> _input_nodes;
    std::vector<std::shared_ptr<INode>> _output_nodes;
    std::vector<std::shared_ptr<INode>> _inter_nodes;
    void add_into_inter(std::shared_ptr<INode> node);

};
} // namespace g