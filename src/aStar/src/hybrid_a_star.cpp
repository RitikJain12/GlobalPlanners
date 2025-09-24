#include "hybrid_a_star.h"

HybridAStar::HybridAStar(std::shared_ptr<Map> map, const float min_velocity, const float theta_resolution)
    : AStar(map, theta_resolution), _min_velocity(min_velocity), _downsampled_map(map, 2)
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

float HybridAStar::DistanceHeuristic(int curr_index, int width, int goal_x, int goal_y)
{
    int dx = static_cast<int>(curr_index % width) - static_cast<int>(goal_x);
    int dy = static_cast<int>(curr_index / width) - static_cast<int>(goal_y);
    return std::sqrt(dx * dx + dy * dy);
}

float HybridAStar::getObstacleHurestic(const Point &point)
{
    int size_x = _downsampled_map.getSizeInX();
    int start_x = floor(point.x / 2.0);
    int start_y = floor(point.y / 2.0);
    int start_index = start_x + (start_y * size_x);
    float &requested_cost = _obstacle_heuristic_map[start_index];
    if (requested_cost > 0.0)
    {
        return 2.0 * requested_cost;
    }

    for (std::pair<float, int> &q : _obstacle_heuristic_queue)
    {
        q.first = -_obstacle_heuristic_map[q.second] + DistanceHeuristic(q.second, size_x, start_x, start_y);
    }

    std::make_heap(
        _obstacle_heuristic_queue.begin(), _obstacle_heuristic_queue.end(),
        ObstacleHeuristicComparator{});

    const int size_x_int = static_cast<int>(size_x);
    const unsigned int size_y = _downsampled_map.getSizeInY();
    const float sqrt_2 = sqrt(2);
    float c_cost, cost, travel_cost, new_cost, existing_cost;
    unsigned int idx, mx, my, mx_idx, my_idx;
    unsigned int new_idx = 0;

    const std::vector<int> neighborhood = {1, -1,                             // left right
                                           size_x_int, -size_x_int,           // up down
                                           size_x_int + 1, size_x_int - 1,    // upper diagonals
                                           -size_x_int + 1, -size_x_int - 1}; // lower diagonals

    while (!_obstacle_heuristic_queue.empty())
    {
        idx = _obstacle_heuristic_queue.front().second;
        std::pop_heap(
            _obstacle_heuristic_queue.begin(), _obstacle_heuristic_queue.end(),
            ObstacleHeuristicComparator{});
        _obstacle_heuristic_queue.pop_back();
        c_cost = _obstacle_heuristic_map[idx];
        if (c_cost > 0.0f)
        {
            // cell has been processed and closed, no further cost improvements
            // are mathematically possible thanks to euclidean distance heuristic consistency
            continue;
        }
        c_cost = -c_cost;
        _obstacle_heuristic_map[idx] = c_cost; // set a positive value to close the cell

        my_idx = idx / size_x;
        mx_idx = idx - (my_idx * size_x);

        // find neighbors
        for (unsigned int i = 0; i != neighborhood.size(); i++)
        {
            new_idx = static_cast<unsigned int>(static_cast<int>(idx) + neighborhood[i]);

            // if neighbor path is better and non-lethal, set new cost and add to queue
            if (new_idx < size_x * size_y)
            {
                cost = static_cast<float>(_downsampled_map.getCost(new_idx));
                if (cost != 0)
                {
                    continue;
                }

                my = new_idx / size_x;
                mx = new_idx - (my * size_x);

                if (mx == 0 && mx_idx >= size_x - 1 || mx >= size_x - 1 && mx_idx == 0)
                {
                    continue;
                }
                if (my == 0 && my_idx >= size_y - 1 || my >= size_y - 1 && my_idx == 0)
                {
                    continue;
                }

                existing_cost = _obstacle_heuristic_map[new_idx];
                if (existing_cost <= 0.0f)
                {
                    travel_cost =
                        ((i <= 3) ? 1.0f : sqrt_2) * (1.0f + (cost / 252.0f));
                    new_cost = c_cost + travel_cost;
                    if (existing_cost == 0.0f || -existing_cost > new_cost)
                    {
                        // the negative value means the cell is in the open set
                        _obstacle_heuristic_map[new_idx] = -new_cost;
                        _obstacle_heuristic_queue.emplace_back(
                            new_cost + DistanceHeuristic(new_idx, size_x, start_x, start_y), new_idx);
                        std::push_heap(
                            _obstacle_heuristic_queue.begin(), _obstacle_heuristic_queue.end(),
                            ObstacleHeuristicComparator{});
                    }
                }
            }
        }

        if (idx == start_index)
        {
            break;
        }
    }

    // return requested_node_cost which has been updated by the search
    // costs are doubled due to downsampling
    return 2.0 * requested_cost;
}

void HybridAStar::resetObstacleHurestic()
{
    int size = _downsampled_map.getSizeInX() * _downsampled_map.getSizeInY();
    if (_obstacle_heuristic_map.size() == size)
    {
        std::fill(_obstacle_heuristic_map.begin(), _obstacle_heuristic_map.end(), 0.0);
    }
    else
    {
        std::fill(_obstacle_heuristic_map.begin(), _obstacle_heuristic_map.end(), 0.0);
        _obstacle_heuristic_map.resize(size, 0.0);
    }
    _obstacle_heuristic_queue.clear();
    _obstacle_heuristic_map.reserve(size);

    int goal_index = floor(_end_point.x / 2.0) + (floor(_end_point.y / 2.0) * _downsampled_map.getSizeInX());
    _obstacle_heuristic_queue.emplace_back(std::pair(Point::euclideanDistance(_end_point, _start_point), goal_index));
    _obstacle_heuristic_map[goal_index] = -0.0001;
}