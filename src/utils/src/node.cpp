#include "node.h"

// Constructor initializes the node with a point and an optional parent
Node::Node(const Point &point, Node *parent)
    : point(point), parent(parent), g(0.0f), h(0.0f), f(0.0f) {}

// Comparison operator for priority queue (min-heap based on f value)
bool Node::operator>(const Node &other) const
{
    return f > other.f;
}

// Equality operator for comparing two Node objects
bool Node::operator==(const Node &other) const
{
    return point == other.point && g <= other.g;
}