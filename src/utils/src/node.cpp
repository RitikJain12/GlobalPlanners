#include "node.h"

// Constructor initializes the node with a point and an optional parent
Node::Node(const Point &point, Node *parent)
    : point(point), parent(parent), g(FLT_MAX), h(FLT_MAX), f(FLT_MAX) {}

// Comparison operator for priority queue (min-heap based on f value)
bool Node::operator>(const Node &other) const
{
    return f > other.f;
}