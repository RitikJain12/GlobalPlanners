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
    AStar(float xy_resolution = 1.0f, float theta_resolution = 8.0f);

    // Destructor
    ~AStar() = default;

    // Method to set the start point
    void setStartNode(const Point &start);

    // Method to set the start point with coordinates
    void setStartNode(float x, float y, float theta);

    // Method to set the end point
    void setEndNode(const Point &end);

    // Method to set the end point with coordinates
    void setEndNode(float x, float y, float theta);

    // Method to set the map
    void setMap(const std::vector<int8_t> &map, int width, int height);

    // Method to get the path
    bool getPath(std::vector<Point> &path);

private:
    // Helper function to round points to the nearest resolution
    void roundPointsToResolution(Point &point);

    // Helper function to get neighbors of a point
    std::vector<Point> getNeighbors(const Point &point);

    // Helper function to backtrack the path from the end node to the start node
    void backtrackPath(std::vector<Point> &path, Node *endNode);

    // Helper function to calculate costs between two nodes
    float calculateCosts(const Node &currentNode, const Node &neighborNode);

    // Helper function to check for collisions
    bool checkCollision(const Point &point);

    // Member variables
    Point _startNode; // Starting point of the path
    Point _endNode;   // Ending point of the path

    float _xy_resolution;     // Resolution for XY coordinates
    float _theta_resolution;  // Resolution for theta
    float _theta_least_count; // Least count for theta resolution

    std::vector<int8_t> _map; // Map representation
    int _mapWidth;            // Width of the map
    int _mapHeight;           // Height of the map
};