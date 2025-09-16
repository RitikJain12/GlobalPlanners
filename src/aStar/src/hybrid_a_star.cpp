#include "hybrid_a_star.h"

HybridAStar::HybridAStar(const float xy_resolution, const float theta_resolution)
    : AStar(xy_resolution, theta_resolution)
{
    _allow_reverse = false;
    _min_velocity = 0.3;
    _wheelbase = 3.0;
}

std::vector<Point> HybridAStar::getNeighbors(const Point &point)
{
    std::vector<Point> neighbors;
    // Generate neighbors based on the heading angle and resolution

    // Add neighbors in forward direction
    for (float dtheta = 1; dtheta < _theta_resolution; dtheta += 1)
    {
        // add kinematics here
        float x = _min_velocity * cos(point.theta);
        float y = _min_velocity * sin(point.theta);
        float theta = _min_velocity * (tan(dtheta * _theta_least_count) / _wheelbase);
        Point neighbor(x, y, theta);
        roundPointsToResolution(neighbor);

        // Check for collision before adding to neighbors
        if (!checkCollision(neighbor))
            neighbors.push_back(neighbor);
    }

    if (_allow_reverse)
    {
        for (float dtheta = 1; dtheta < _theta_resolution; dtheta += 1)
        {
            // add kinematics here
            float x = (-_min_velocity) * cos(point.theta);
            float y = (-_min_velocity) * sin(point.theta);
            float theta = (-_min_velocity) * (tan(dtheta * _theta_least_count) / _wheelbase);
            Point neighbor(x, y, theta);
            roundPointsToResolution(neighbor);

            // Check for collision before adding to neighbors
            if (!checkCollision(neighbor))
                neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}