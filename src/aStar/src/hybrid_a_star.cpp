#include "hybrid_a_star.h"

HybridAStar::HybridAStar(const float min_velocity, const float theta_resolution)
    : AStar(theta_resolution), _min_velocity(min_velocity)
{
    _allow_reverse = false;
    _wheelbase = 2.0;
    _steer_resolution = M_PI / 8.0;
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
    float angleDifference = Point::absDiff(currentNode.point.theta, neighborNode.point.theta);
    Point::roundTheta(angleDifference);
    return distance + (angleDifference / _theta_least_count);
}

float HybridAStar::calculateHeuristic(const Node &currentNode)
{
    // Using Euclidean distance as heuristic
    float dist = Point::euclideanDistance(currentNode.point, _end_point);
    float angle_diff = Point::absDiff(Point::slope(currentNode.point, _end_point), currentNode.point.theta);
    Point::roundTheta(angle_diff);
    return dist + (angle_diff / _theta_least_count);
}