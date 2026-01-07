#pragma once

#include <float.h>
#include <math.h>

#include <unordered_set>

#include "point.h"

class Node {
 public:
  // Constructor initializes the node with a point and an optional parent
  Node(const Point& point, Node* parent = nullptr);

  // Member variables
  Point point;   // The point represented by this node
  Node* parent;  // Pointer to the parent node
  float g;       // Cost from start to this node
  float h;       // Heuristic cost from this node to end
  float f;       // Total cost (g + h)
};

namespace std {
template <>
struct hash<Node> {
  std::size_t operator()(const Node& node) const {
    return std::hash<float>()(node.point.x) ^ std::hash<float>()(node.point.y) ^
           std::hash<float>()(node.point.theta);
  }
};
}  // namespace std