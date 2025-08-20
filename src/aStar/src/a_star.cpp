#include "a_star.h"

AStar::AStar(float xy_resolution, float theta_resolution)
    : _startNode(Point()), _endNode(Point()),
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

void AStar::setStartNode(const Point &start)
{
    _startNode = Point(start);
    roundPointsToResolution(_startNode);
}

void AStar::setStartNode(float x, float y, float theta)
{
    _startNode = Point(x, y, theta);
    roundPointsToResolution(_startNode);
}

void AStar::setEndNode(const Point &end)
{
    _endNode = Point(end);
    roundPointsToResolution(_endNode);
}

void AStar::setEndNode(float x, float y, float theta)
{
    _endNode = Point(x, y, theta);
    roundPointsToResolution(_endNode);
}

void AStar::setMap(const std::vector<int8_t> &map, int width, int height)
{
    _map = map;
    _mapWidth = width / _xy_resolution;   // Convert width to number of grid cells
    _mapHeight = height / _xy_resolution; // Convert height to number of grid cells

    _node_data.reserve(_mapWidth * _mapHeight * _theta_resolution);             // Reserve space for nodes
    _node_position.resize(_mapWidth * _mapHeight * _theta_resolution, nullptr); // Initialize node pointers
}

void AStar::setNodeAtPose(const Point &point, Node *node)
{
    int index = (static_cast<int>(point.theta / _theta_least_count)) +
                (static_cast<int>(point.x / _xy_resolution) * _theta_resolution) +
                (static_cast<int>(point.y / _xy_resolution) * _mapWidth * _theta_resolution);
    _node_position[index] = node;
}

Node *AStar::getNodeAtPose(const Point &point)
{
    int index = (static_cast<int>(point.theta / _theta_least_count)) +
                (static_cast<int>(point.x / _xy_resolution) * _theta_resolution) +
                (static_cast<int>(point.y / _xy_resolution) * _mapWidth * _theta_resolution);
    return _node_position[index];
}

bool AStar::checkCollision(const Point &point)
{
    // Convert point coordinates to grid indices
    int index_x = static_cast<int>(point.x / _xy_resolution);
    int index_y = static_cast<int>(point.y / _xy_resolution);

    // Check if the point is within the bounds of the map
    if (index_x < 0 || index_x >= _mapWidth || index_y < 0 || index_y >= _mapHeight)
        return true; // Out of bounds

    // Check if the point collides with an obstacle in the map
    int index = index_y * _mapWidth + index_x;
    return _map[index] != 0;
}

std::vector<Point> AStar::getNeighbors(const Point &point)
{
    std::vector<Point> neighbors;
    // Generate neighbors based on the heading angle and resolution

    // Add neighbors in the theta direction
    for (float dtheta = _theta_least_count; dtheta < (2 * M_PI); dtheta += _theta_least_count)
    {
        Point neighbor(point.x, point.y, (point.theta + dtheta));
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

float AStar::calculateCosts(const Node &currentNode, const Node &neighborNode)
{
    float distance = Point::euclideanDistance(currentNode.point, neighborNode.point);
    float angleDifference = std::abs(currentNode.point.theta - neighborNode.point.theta) / _theta_least_count;
    return distance + angleDifference;
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
    if (_map.empty() || _mapWidth == 0 || _mapHeight == 0)
    {
        return false; // No map set
    }

    if (_startNode == _endNode)
    {
        path.push_back(_startNode);
        return true; // Start and end points are the same
    }

    std::priority_queue<Node *, std::vector<Node *>, std::greater<Node *>> openList;

    int node_data_index = 0;

    Node startNode = Node(_startNode);
    startNode.g = 0.0f; // Cost from start to start is zero
    startNode.h = Point::euclideanDistance(_startNode, _endNode);
    startNode.f = startNode.g + startNode.h;

    _node_data[node_data_index] = startNode;
    setNodeAtPose(_startNode, &_node_data[node_data_index]);
    openList.push(&_node_data[node_data_index]);
    node_data_index++;

    while (!openList.empty())
    {
        Node *currentNode = openList.top();
        openList.pop();

        // Check if we reached the end node
        if (currentNode->point == _endNode)
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
            float temp_g = currentNode->g + calculateCosts(*currentNode, *neighbor);
            float temp_h = Point::euclideanDistance(neighbor->point, _endNode);
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