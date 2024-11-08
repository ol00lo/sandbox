#ifndef POWER_OP_HPP
#define POWER_OP_HPP
#include "i_functions_node.hpp"

namespace g
{
namespace op
{
std::shared_ptr<IFunctionalNode> sqr(std::shared_ptr<INode> a);
} // namespace op
} // namespace g
#endif