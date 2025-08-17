#pragma once

#include "point.h"
#include <math.h>

class Node
{
public:
    // Constructor initializes the node with a point and an optional parent
    Node(const Point &point, Node *parent = nullptr);

    // Member variables
    Point point;  // The point represented by this node
    Node *parent; // Pointer to the parent node
    float g;      // Cost from start to this node
    float h;      // Heuristic cost from this node to end
    float f;      // Total cost (g + h)

    // Comparison operator for priority queue (min-heap based on f value)
    bool operator>(const Node &other) const;
};