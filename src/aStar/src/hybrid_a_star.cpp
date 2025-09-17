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

    float dx = _min_velocity * cos(point.theta);
    float dy = _min_velocity * sin(point.theta);

    // Add neighbors in forward direction
    for (float steer = -2; steer <= 2; steer += 1)
    {
        float dtheta = _min_velocity * (tan(steer * _steer_resolution) / _wheelbase);
        float theta_dash = point.theta + dtheta;
        Point::roundTheta(theta_dash);
        Point neighbor((point.x + dx), (point.y + dy), theta_dash);
        // roundPointsToResolution(neighbor);

        // Check for collision before adding to neighbors
        if (!checkCollision(neighbor))
            neighbors.push_back(neighbor);
    }

    if (_allow_reverse)
    {
        for (float steer = -1; steer <= 1; steer += 1)
        {
            float dtheta = _min_velocity * (tan(steer * _steer_resolution) / _wheelbase);
            Point neighbor((point.x - dx), (point.y - dy), (point.theta - dtheta));

            // Check for collision before adding to neighbors
            if (!checkCollision(neighbor))
                neighbors.push_back(neighbor);
        }
    }

    return neighbors;
}

float HybridAStar::calculateTravelCost(const Node &currentNode, const Node &neighborNode)
{
    float distance = Point::euclideanDistance(currentNode.point, neighborNode.point);
    float angleDifference = std::abs(currentNode.point.theta - neighborNode.point.theta) / _theta_least_count;
    return distance + angleDifference;
}

float HybridAStar::calculateHeuristic(const Node &currentNode)
{
    // Using Euclidean distance as heuristic
    return Point::euclideanDistance(currentNode.point, _end_point);
}