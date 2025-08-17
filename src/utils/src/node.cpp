#include "node.h"

// Constructor initializes the node with a point and an optional parent
Node::Node(const Point &point, Node *parent)
    : point(point), parent(parent), g(0.0f), h(0.0f), f(0.0f) {}

bool Node::operator>(const Node &other) const
{
    return f > other.f;
}