#ifndef ARITHMETIC_OP_HPP
#define ARITHMETIC_OP_HPP
#include "i_functions_node.hpp"

namespace g
{
namespace op
{
std::shared_ptr<IFunctionalNode> mult(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
std::shared_ptr<IFunctionalNode> plus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
std::shared_ptr<IFunctionalNode> minus(std::shared_ptr<INode> a1, std::shared_ptr<INode> a2);
} // namespace op
} // namespace g
#endif