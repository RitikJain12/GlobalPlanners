#include "a_star.h"

AStar::AStar(const float xy_resolution, const float theta_resolution)
    : _start_point(Point()), _end_point(Point()),
      _xy_resolution(xy_resolution), _theta_resolution(theta_resolution)
{
    _theta_least_count = (2 * M_PI) / _theta_resolution; // Convert resolution to radians

    Point::setThreshold(_xy_resolution / 2.0);          // Set XY threshold
    Point::setThetaThreshold(_theta_least_count / 2.0); // Set theta threshold
}

void AStar::roundPointsToResolution(Point &point)
{
    point.x = std::round(point.x / _xy_resolution) * _xy_resolution;
    point.y = std::round(point.y / _xy_resolution) * _xy_resolution;

    while (point.theta < 0 || point.theta >= (2 * M_PI))
    {
        if (point.theta < 0)
            point.theta += (2 * M_PI); // Normalize theta to be within [0, 2π]
        else if (point.theta >= (2 * M_PI))
            point.theta -= (2 * M_PI);
    }
    point.theta = std::round(point.theta / _theta_least_count) * _theta_least_count;
}

void AStar::setStartPoint(const Point &start)
{
    _start_point = Point(start);
    roundPointsToResolution(_start_point);
}

void AStar::setStartPoint(float x, float y, float theta)
{
    _start_point = Point(x, y, theta);
    roundPointsToResolution(_start_point);
}

void AStar::setGoal(const Point &end)
{
    _end_point = Point(end);
    roundPointsToResolution(_end_point);
}

void AStar::setGoal(float x, float y, float theta)
{
    _end_point = Point(x, y, theta);
    roundPointsToResolution(_end_point);
}

void AStar::setMap(const std::vector<int8_t> &map, int width, int height)
{
    _map = map;
    _map_width = width;
    _map_height = height;
    _grid_width = width / _xy_resolution;   // Convert width to number of grid cells
    _grid_height = height / _xy_resolution; // Convert height to number of grid cells

    _node_data.reserve(_grid_width * _grid_height * _theta_resolution);             // Reserve space for nodes
    _node_position.resize(_grid_width * _grid_height * _theta_resolution, nullptr); // Initialize node pointers
}

void AStar::setNodeAtPose(Point point, Node *node)
{
    roundPointsToResolution(point);
    int index = (static_cast<int>(point.theta / _theta_least_count)) +
                (static_cast<int>(point.x / _xy_resolution) * _theta_resolution) +
                (static_cast<int>(point.y / _xy_resolution) * _grid_width * _theta_resolution);
    _node_position[index] = node;
}

Node *AStar::getNodeAtPose(Point point)
{
    roundPointsToResolution(point);
    int index = (static_cast<int>(point.theta / _theta_least_count)) +
                (static_cast<int>(point.x / _xy_resolution) * _theta_resolution) +
                (static_cast<int>(point.y / _xy_resolution) * _grid_width * _theta_resolution);
    return _node_position[index];
}

bool AStar::checkCollision(const Point &point)
{
    if (point.x < 0 || point.x > _map_width || point.y < 0 || point.y > _map_height)
        return true;

    // Convert point coordinates to grid indices
    int index_x = static_cast<int>(point.x / _xy_resolution);
    int index_y = static_cast<int>(point.y / _xy_resolution);

    // Check if the point is within the bounds of the map
    if (index_x < 0 || index_x >= _grid_width || index_y < 0 || index_y >= _grid_height)
        return true; // Out of bounds

    // Check if the point collides with an obstacle in the map
    int index = index_y * _grid_width + index_x;
    return _map[index] != 0;
}

std::vector<Point> AStar::getNeighbors(const Point &point)
{
    std::vector<Point> neighbors;
    // Generate neighbors based on the heading angle and resolution

    // Add neighbors in the theta direction
    for (float dtheta = 1; dtheta < _theta_resolution; dtheta += 1)
    {
        Point neighbor(point.x, point.y, (point.theta + (dtheta * _theta_least_count)));
        roundPointsToResolution(neighbor);
        neighbors.push_back(neighbor);
    }

    // Add neighbors in the XY plane
    Point neighbor(Point(point.x + _xy_resolution * cos(point.theta), point.y + _xy_resolution * sin(point.theta), point.theta));
    roundPointsToResolution(neighbor);

    // Check for collision before adding to neighbors
    if (!checkCollision(neighbor))
        neighbors.push_back(neighbor);

    return neighbors;
}

float AStar::calculateTravelCost(const Node &currentNode, const Node &neighborNode)
{
    float distance = Point::euclideanDistance(currentNode.point, neighborNode.point);
    float angleDifference = std::abs(currentNode.point.theta - neighborNode.point.theta) / _theta_least_count;
    return distance + angleDifference;
}

float AStar::calculateHeuristic(const Node &currentNode)
{
    // Using Euclidean distance as heuristic
    return Point::euclideanDistance(currentNode.point, _end_point);
}

void AStar::backtrackPath(std::vector<Point> &path, Node *currentNode)
{
    while (currentNode != nullptr)
    {
        path.push_back(currentNode->point);
        currentNode = currentNode->parent;
    }
    std::reverse(path.begin(), path.end()); // Reverse the path to get it from start to end
}

bool AStar::getPath(std::vector<Point> &path)
{
    if (_map.empty() || _grid_width == 0 || _grid_height == 0)
    {
        return false; // No map set
    }

    if (_start_point == _end_point)
    {
        path.push_back(_start_point);
        return true; // Start and end points are the same
    }

    std::priority_queue<Node *, std::vector<Node *>, CompareNode> openList;

    int node_data_index = 0;

    Node startNode = Node(_start_point);
    startNode.g = 0.0f; // Cost from start to start is zero
    startNode.h = calculateHeuristic(startNode);
    startNode.f = startNode.g + startNode.h;

    _node_data[node_data_index] = startNode;
    setNodeAtPose(_start_point, &_node_data[node_data_index]);
    openList.push(&_node_data[node_data_index]);
    node_data_index++;

    while (!openList.empty())
    {
        Node *currentNode = openList.top();
        openList.pop();

        // Check if we reached the end node
        if (inTollerance(currentNode->point))
        {
            // Reconstruct the path
            backtrackPath(path, currentNode);
            return true; // Path found
        }

        // Get neighbors of the current node
        std::vector<Point> neighbors = getNeighbors(currentNode->point);
        for (const Point &neighborPoint : neighbors)
        {
            Node *neighbor = getNodeAtPose(neighborPoint);

            if (neighbor == nullptr)
            {
                // Create a new node if it doesn't exist
                _node_data[node_data_index] = Node(neighborPoint);
                neighbor = &_node_data[node_data_index];
                node_data_index++;
                setNodeAtPose(neighborPoint, neighbor);
            }

            // Calculate costs
            float temp_g = currentNode->g + calculateTravelCost(*currentNode, *neighbor);
            float temp_h = calculateHeuristic(*neighbor);
            float temp_f = temp_g + temp_h;

            if (temp_f < neighbor->f)
            {
                // Update the neighbor node with new costs
                neighbor->g = temp_g;
                neighbor->h = temp_h;
                neighbor->f = temp_f;
                neighbor->parent = currentNode;

                openList.push(neighbor);
            }
        }
    }

    return false; // No path found
}

bool AStar::inTollerance(const Point &point)
{
    float dist = Point::euclideanDistance(point, _end_point);
    float theta_diff = abs(point.theta - _end_point.theta);

    if (dist <= _xy_resolution && theta_diff <= _theta_least_count)
    {
        return true;
    }

    return false;
}