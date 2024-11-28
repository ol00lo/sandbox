#include "model.hpp"
#include "arithmetic_nodes.hpp"
#include "power_nodes.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>

using namespace g;

Model::Model(std::vector<std::shared_ptr<InputNode>> inputs, std::vector<std::shared_ptr<INode>> outputs)
    : _input_nodes(std::move(inputs)), _output_nodes(std::move(outputs))
{
    for (const auto& output : _output_nodes)
    {
        add_into_inter(output);
        _output_nodes_names.push_back("InterNode[" + std::to_string(_inter_nodes_names.size() - 1) + "]");
    }
}

std::vector<double> Model::compute(const std::vector<double>& input_values)
{
    if (input_values.size() != _input_nodes.size())
    {
        throw std::runtime_error("Size of input values does not match the number of input nodes.");
    }
    for (size_t i = 0; i < _input_nodes.size(); ++i)
    {
        _input_nodes[i]->set_value(input_values[i]);
    }

    std::vector<double> results;
    for (const auto& output : _output_nodes)
    {
        results.push_back(output->get_value());
    }
    return results;
}

void Model::add_into_inter(std::shared_ptr<INode> node)
{
    if (node->classname() == "InputNode")
    {
        return;
    }

    std::string dependencies;
    for (const auto& prev : node->get_prev())
    {
        add_into_inter(prev);
    }
    for (const auto& prev : node->get_prev())
    {
        if (prev->classname() == "InputNode")
        {
            auto it = std::find(_input_nodes.begin(), _input_nodes.end(), prev);
            if (it != _input_nodes.end())
            {
                size_t idx = std::distance(_input_nodes.begin(), it);
                dependencies += "InputNode[" + std::to_string(idx) + "] ";
            }
        }
        else
        {
            dependencies += "InterNode[" + std::to_string(_inter_nodes_names.size() - 1) + "] ";
        }
    }
    _inter_nodes_names.push_back(node->classname() + "\t" + dependencies);
}

void Model::save(const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open file for writing.");
    }
    for (const auto& input : _input_nodes)
    {
        file << input->serialize() << std::endl;
    }
    for (size_t i = 0; i < _inter_nodes_names.size(); ++i)
    {
        file << "InterNode " << _inter_nodes_names[i] << std::endl;
    }
    for (const auto& name : _output_nodes_names)
    {
        file << "OutputNode " << name << std::endl;
    }
    file.close();
}

int extractIndex(const std::string& token)
{
    auto start_pos = token.find('[');
    auto end_pos = token.find(']');

    if (start_pos != std::string::npos && end_pos != std::string::npos && start_pos < end_pos)
    {
        return std::stoi(token.substr(start_pos + 1, end_pos - start_pos - 1));
    }
    throw std::runtime_error("Invalid token format: " + token);
}

Model Model::load(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("Cannot open file for reading.");
    }
    std::vector<std::shared_ptr<InputNode>> input_nodes;
    std::vector<std::shared_ptr<INode>> nodes;
    std::vector<std::shared_ptr<INode>> output_nodes;

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string nodeType;
        iss >> nodeType;

        if (nodeType == "InputNode")
        {
            double value;
            iss >> value;
            auto input_node = std::make_shared<InputNode>();
            input_node->set_value(value);
            input_nodes.push_back(input_node);
        }
        else if (nodeType == "InterNode")
        {
            std::string operation;
            iss >> operation;

            std::vector<std::shared_ptr<INode>> prev;
            std::string token;
            while (iss >> token)
            {
                if (token.find("InputNode[") != std::string::npos)
                {
                    prev.push_back(input_nodes[extractIndex(token)]);
                }
                else if (token.find("InterNode[") != std::string::npos)
                {
                    prev.push_back(nodes[extractIndex(token)]);
                }
                else
                {
                    throw std::runtime_error("Invalid token format: " + token);
                }
            }

            std::shared_ptr<IFunctionalNode> inter_node;
            if (operation == "PlusNode")
            {
                inter_node = op::plus(prev[0], prev[1]);
            }
            else if (operation == "MinusNode")
            {
                inter_node = op::minus(prev[0], prev[1]);
            }
            else if (operation == "MultNode")
            {
                inter_node = op::mult(prev[0], prev[1]);
            }
            else if (operation == "SqrNode")
            {
                inter_node = op::sqr(prev[0]);
            }
            else
            {
                throw std::runtime_error("Unknown operation: " + operation);
            }
            nodes.push_back(inter_node);
        }
        else if (nodeType == "OutputNode")
        {
            std::string token;
            while (iss >> token)
            {
                int idx = extractIndex(token);
                output_nodes.push_back(nodes[idx]);
            }
        }
    }

    file.close();
    return Model(input_nodes, output_nodes);
}