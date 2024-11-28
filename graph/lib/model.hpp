#include "i_node.hpp"
#include "input_node.hpp"

namespace g
{
class Model
{
public:
    Model(std::vector<std::shared_ptr<InputNode>> inputs, std::vector<std::shared_ptr<INode>> outputs);
    void save(const std::string& filename);
    std::vector<double> compute(const std::vector<double>& input_values);
    static Model load(const std::string& filename);

private:
    std::vector<std::shared_ptr<InputNode>> _input_nodes;
    std::vector<std::shared_ptr<INode>> _output_nodes;
    std::vector<std::string> _inter_nodes_names = {};
    std::vector<std::string> _output_nodes_names = {};
    void add_into_inter(std::shared_ptr<INode> node);

};
} // namespace g