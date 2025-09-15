#include "hybrid_a_star.h"

HybridAStar::HybridAStar(const float xy_resolution, const float theta_resolution)
    : AStar(xy_resolution, theta_resolution)
{
    _allow_reverse = false;
}

std::vector<Point> HybridAStar::getNeighbors(const Point &point)
{
    std::vector<Point> neighbors;
    // Generate neighbors based on the heading angle and resolution

    // Add neighbors in forward direction
    for (float dtheta = 1; dtheta < _theta_resolution; dtheta += 1)
    {
        // add kinematics here 
        float x = 0;
        float y = 0;
        float theta = 0;
        Point neighbor(x, y, theta);
        roundPointsToResolution(neighbor);
        // Check for collision before adding to neighbors
        if (!checkCollision(neighbor))
            neighbors.push_back(neighbor);
    }

    if (_allow_reverse)
    {
    }

    return neighbors;
}