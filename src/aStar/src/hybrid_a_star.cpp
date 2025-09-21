#include "hybrid_a_star.h"

HybridAStar::HybridAStar(std::shared_ptr<Map> map, const float min_velocity, const float theta_resolution)
    : AStar(map, theta_resolution), _min_velocity(min_velocity)
{
    _allow_reverse = false;
    _wheelbase = 2.0;
    _steer_resolution = (2 * M_PI) / 64.0; // 5.625 deg
    _max_steer = (3 * M_PI) / 16.0;        // 33.75 deg
    _steer_step = static_cast<int>(_max_steer / _steer_resolution);
    _max_turnning_radius = _wheelbase / tan(_max_steer);
}

std::vector<Point> HybridAStar::getNeighbors(const Point &point)
{
    std::vector<Point> neighbors;
    // Generate neighbors based on the heading angle and resolution

    float dx = _min_velocity * cos(point.theta);
    float dy = _min_velocity * sin(point.theta);

    // Add neighbors in forward direction
    for (float steer = -_steer_step; steer <= _steer_step; steer += 1)
    {
        float dtheta = _min_velocity * (tan(steer * _steer_resolution) / _wheelbase);
        float theta_dash = point.theta + dtheta;
        Point::normalizeTheta(theta_dash);
        Point neighbor((point.x + dx), (point.y + dy), theta_dash, (steer * _steer_resolution));

        // Check for collision before adding to neighbors
        if (!checkCollision(neighbor))
            neighbors.push_back(neighbor);
    }

    if (_allow_reverse)
    {
        for (float steer = -_steer_step; steer <= _steer_step; steer += 1)
        {
            float dtheta = _min_velocity * (tan(steer * _steer_resolution) / _wheelbase);
            float theta_dash = point.theta - dtheta;
            Point::normalizeTheta(theta_dash);
            Point neighbor((point.x - dx), (point.y - dy), theta_dash, (steer * _steer_resolution));
            neighbor.reverse = true;

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
    float angleDifference = Point::absAngleDiff(currentNode.point.steer, neighborNode.point.steer);
    Point::normalizeTheta(angleDifference);
    int reverse_penalty = 0;
    if (neighborNode.point.reverse)
    {
        reverse_penalty = 10;
    }
    return distance + (angleDifference / _theta_least_count) + reverse_penalty;
}

float HybridAStar::calculateHeuristic(const Node &currentNode)
{
    float dist_heuristic = getDistanceHurestic(currentNode.point);
    float obs_heuristic = getObstacleHurestic(currentNode.point);
    return std::max(dist_heuristic, obs_heuristic);
}

float HybridAStar::getDistanceHurestic(const Point &point)
{
    DubinsPath path;
    double q0[3] = {point.x, point.y, point.theta};
    double q1[3] = {_end_point.x, _end_point.y, _end_point.theta};
    if (dubins_shortest_path(&path, q0, q1, _max_turnning_radius) == 0)
    {
        return static_cast<float>(dubins_path_length(&path));
    }
    return -1;
}

float HybridAStar::getObstacleHurestic(const Point &point)
{
    // ToDo
    return 0;
}