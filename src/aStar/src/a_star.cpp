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
    return Point::euclideanDistance(currentNode.point, neighborNode.point) + std::abs(currentNode.point.theta - neighborNode.point.theta) / _theta_least_count;
}

void AStar::backtrackPath(std::vector<Point> &path)
{
    Node *currentNode = &_endNode;

    while (currentNode != nullptr)
    {
        path.push_back(currentNode->point);
        currentNode = currentNode->parent;
    }
    std::reverse(path.begin(), path.end()); // Reverse the path to get it from start to end
}

bool AStar::getPath(std::vector<Point> &path)
{
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::unordered_set<Node> openSet;
    std::unordered_set<Node> closedSet;

    openList.push(_startNode);
    openSet.insert(_startNode);

    while (!openList.empty())
    {
        Node currentNode = openList.top();
        openList.pop();
        openSet.erase(currentNode);

        // Check if we reached the end node
        if (currentNode.point == _endNode.point)
        {
            // Reconstruct the path
            backtrackPath(path);
            return true; // Path found
        }

        closedSet.insert(currentNode);

        // Get neighbors of the current node
        std::vector<Point> neighbors = getNeighbors(currentNode.point);
        for (const Point &neighborPoint : neighbors)
        {
            Node neighbor(neighborPoint, &currentNode);

            // Skip if neighbor is already in closed set
            if (closedSet.find(neighbor) != closedSet.end())
                continue;

            // Calculate costs
            neighbor.g = currentNode.g + calculateCosts(currentNode, neighbor);
            neighbor.h = Point::euclideanDistance(neighbor.point, _endNode.point);
            neighbor.f = neighbor.g + neighbor.h;

            // If neighbor is not in open set, add it
            if (openSet.find(neighbor) == openSet.end())
            {
                openList.push(neighbor);
                openSet.insert(neighbor);
            }
        }
    }

    return false; // No path found
}