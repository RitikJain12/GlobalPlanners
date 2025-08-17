#include "a_star.h"

AStar::AStar(float xy_resolution, float theta_resolution)
    : startNode(Point()), endNode(Point()),
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
    point.theta = std::round(point.theta / _theta_resolution) * _theta_resolution;
}

void AStar::setStartNode(const Point &start)
{
    startNode = Node(start);
    roundPointsToResolution(startNode.point);
}

void AStar::setStartNode(float x, float y, float theta)
{
    startNode = Node(Point(x, y, theta));
    roundPointsToResolution(startNode.point);
}

void AStar::setEndNode(const Point &end)
{
    endNode = Node(end);
    roundPointsToResolution(endNode.point);
}

void AStar::setEndNode(float x, float y, float theta)
{
    endNode = Point(x, y, theta);
    roundPointsToResolution(endNode.point);
}

std::vector<Point> AStar::getNeighbors(const Point &point)
{
    std::vector<Point> neighbors;
    // Generate neighbors based on the heading angle and resolution

    // Add neighbors in the theta direction
    for (float dtheta = 0; dtheta <= 2 * M_PI; dtheta += ((2 * M_PI) / _theta_resolution))
    {
        if (dtheta == 0.0f)
            continue; // Skip the current point

        Point neighbor(point.x, point.y, point.theta + dtheta);
        roundPointsToResolution(neighbor);
        neighbors.push_back(neighbor);
    }

    // Add neighbors in the XY plane
    Point neighbor(Point(point.x + _xy_resolution * cos(point.theta), point.y + _xy_resolution * sin(point.theta), point.theta));
    roundPointsToResolution(neighbor);
    neighbors.push_back(neighbor);

    return neighbors;
}

float AStar::calculateCosts(const Node &currentNode, const Node &neighborNode)
{
    return Point::euclideanDistance(currentNode.point, neighborNode.point) + std::abs(currentNode.point.theta - neighborNode.point.theta) / _theta_least_count;
}

void AStar::backtrackPath(std::vector<Point> &path)
{
    Node *currentNode = &endNode;

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

    openList.push(startNode);
    openSet.insert(startNode);

    while (!openList.empty())
    {
        Node currentNode = openList.top();
        openList.pop();
        openSet.erase(currentNode);

        // Check if we reached the end node
        if (currentNode.point == endNode.point) 
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
            neighbor.h = Point::euclideanDistance(neighbor.point, endNode.point);
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