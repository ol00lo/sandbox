#include "power_nodes.hpp"
using namespace g;

double SqrNode::compute_value()
{
    double value = _prev_nodes[0]->get_value();
    return value * value;
}