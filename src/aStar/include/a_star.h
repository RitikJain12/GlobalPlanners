#pragma once

#include <iostream>
#include <math.h>
#include <vector>
#include <queue>
#include <unordered_set>
#include <functional>
#include <stdint.h>
#include "point.h"
#include "node.h"

class AStar
{
public:
    // Constructor
    AStar(const float xy_resolution = 1.0f, const float theta_resolution = 8.0f);

    // Destructor
    ~AStar() = default;

    // Method to set the start point
    void setStartPoint(const Point &start);

    // Method to set the start point with coordinates
    void setStartPoint(float x, float y, float theta);

    // Method to set the end point
    void setGoal(const Point &end);

    // Method to set the end point with coordinates
    void setGoal(float x, float y, float theta);

    // Method to set the map
    void setMap(const std::vector<int8_t> &map, int width, int height);

    // Method to get the path
    virtual bool getPath(std::vector<Point> &path);

private:
    // Helper function to calculate costs between two nodes
    float calculateTravelCost(const Node &currentNode, const Node &neighborNode);

    // Helper function to calculate heuristic costs for a node
    float calculateHeuristic(const Node &currentNode);

    // Helper function to get neighbors of a point
    std::vector<Point> getNeighbors(const Point &point);

protected:
    // Helper function to round points to the nearest resolution
    void roundPointsToResolution(Point &point);

    // Helper function to backtrack the path from the end node to the start node
    void backtrackPath(std::vector<Point> &path, Node *endNode);

    // Helper function to check for collisions
    bool checkCollision(const Point &point);

    // Helper function to set a node at a specific pose
    void setNodeAtPose(const Point &point, Node *node);

    // Helper function to get a node at a specific pose
    Node *getNodeAtPose(const Point &point);

    // Member variables
    Point _start_point; // Starting point of the path
    Point _end_point;   // Ending point of the path

    float _xy_resolution;     // Resolution for XY coordinates
    float _theta_resolution;  // Resolution for theta
    float _theta_least_count; // Least count for theta resolution

    std::vector<int8_t> _map; // Map representation
    int _map_width;           // Width of the map
    int _map_height;          // Height of the map

    std::vector<Node> _node_data;       // List of nodes used in the algorithm
    std::vector<Node *> _node_position; // Pointers to nodes for quick access
};