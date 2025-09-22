#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include <queue>
#include <unordered_set>
#include <functional>
#include <memory>
#include <stdint.h>
#include "point.h"
#include "node.h"
#include "map.h"

class AStar
{
public:
    // Constructor
    AStar(std::shared_ptr<Map> map, const float theta_resolution = 8.0f);

    // Destructor
    virtual ~AStar() = default;

    // Method to set the start point
    void setStartPoint(const Point &start);

    // Method to set the start point with coordinates
    void setStartPoint(float x, float y, float theta);

    // Method to set the end point
    void setGoal(const Point &end);

    // Method to set the end point with coordinates
    void setGoal(float x, float y, float theta);

    // Method to get the path
    bool getPath(std::vector<Point> &path);

private:
    // Helper function to backtrack the path from the end node to the start node
    void backtrackPath(std::vector<Point> &path, Node *endNode);

    // Helper function to set a node at a specific pose
    void setNodeAtPose(Point point, Node *node);

    // Helper function to get a node at a specific pose
    Node *getNodeAtPose(Point point);

    // Check for tollerace
    bool inTollerance(const Point &point);

    float _xy_tollerance;
    float _theta_tollerance;
    std::vector<Node> _node_data;       // List of nodes used in the algorithm
    std::vector<Node *> _node_position; // Pointers to nodes for quick access
    std::shared_ptr<Map> _map;

protected:
    // Helper function to round points to the nearest resolution
    // void roundPointsToResolution(Point &point);

    // Helper function to check for collisions
    bool checkCollision(Point point);

    // Helper function to calculate costs between two nodes
    virtual float calculateTravelCost(const Node &currentNode, const Node &neighborNode);

    // Helper function to calculate heuristic costs for a node
    virtual float calculateHeuristic(const Node &currentNode);

    // Helper function to get neighbors of a point
    virtual std::vector<Point> getNeighbors(const Point &point);

    // Member variables
    Point _start_point; // Starting point of the path
    Point _end_point;   // Ending point of the path

    float _xy_resolution;     // Resolution for XY coordinates
    float _theta_resolution;  // Resolution for theta
    float _theta_least_count; // Least count for theta resolution

};

struct CompareNode
{
    bool operator()(const Node *a, const Node *b)
    {
        // For a min-priority queue, return true if 'a' has a GREATER value than 'b'
        // (so 'b' is considered "smaller" and will be at the top)
        return a->f > b->f;
    }
};