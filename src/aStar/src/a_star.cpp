#include "a_star.h"

AStar::AStar(std::shared_ptr<Map> map, const float theta_resolution,
             const float xy_tollerance, const float theta_tollerance)
    : _start_point(Point()), _end_point(Point()),
      _theta_resolution(ceil(theta_resolution)),
      _xy_tollerance(xy_tollerance),
      _theta_tollerance(theta_tollerance),
      _map(map)
{
    _theta_least_count = (2 * M_PI) / _theta_resolution; // Convert resolution to radians
    int grid_width;
    int grid_height;

    _map->getMapDimentions(grid_width, grid_height, _xy_resolution);

    _node_data.reserve((unsigned long int)grid_width * grid_height * _theta_resolution);             // Reserve space for nodes
    _node_position.resize((unsigned long int)grid_width * grid_height * _theta_resolution, nullptr); // Initialize node pointers

    Point::setLeastCount(_xy_resolution, _theta_least_count); // Set least count for Point class
}

void AStar::setStartPoint(const Point &start)
{
    _start_point = Point(start);
    Point::normalizePoint(_start_point);
}

void AStar::setStartPoint(float x, float y, float theta)
{
    _start_point = Point(x, y, theta);
    Point::normalizePoint(_start_point);
}

void AStar::setGoal(const Point &end)
{
    _end_point = Point(end);
    Point::normalizePoint(_end_point);
}

void AStar::setGoal(float x, float y, float theta)
{
    _end_point = Point(x, y, theta);
    Point::normalizePoint(_end_point);
}

void AStar::setNodeAtPose(Point point, Node *node)
{
    // roundPointsToResolution(point);
    int index_xy = _map->getIndex(point.x, point.y);
    unsigned long int index = static_cast<int>(point.theta / _theta_least_count) + (index_xy * _theta_resolution);
    _node_position[index] = node;
}

Node *AStar::getNodeAtPose(Point point)
{
    // roundPointsToResolution(point);
    int index_xy = _map->getIndex(point.x, point.y);
    unsigned long int index = static_cast<int>(point.theta / _theta_least_count) + (index_xy * _theta_resolution);
    return _node_position[index];
}

bool AStar::checkCollision(Point point)
{
    int index_x, index_y;
    if (!_map->getWorldtoMap(index_x, index_y, point.x, point.y))
        return true;
    if (_map->getCost(index_x, index_y) != 0)
        return true;

    std::vector<std::pair<int, int>> fp_points = _map->getFootprintCells(point);

    for (const std::pair<int, int> &p : fp_points)
    {
        index_x = p.first;
        index_y = p.second;
        if (_map->getCost(index_x, index_y) != 0)
            return true;
    }

    return false;
}

std::vector<Point> AStar::getNeighbors(const Point &point)
{
    std::vector<Point> neighbors;
    // Generate neighbors based on the heading angle and resolution

    // Add neighbors in the theta direction
    for (float dtheta = 1; dtheta < _theta_resolution; dtheta += 1)
    {
        float theta_dash = point.theta + (dtheta * _theta_least_count);
        Point::normalizeTheta(theta_dash);
        Point neighbor(point.x, point.y, theta_dash);
        Point::normalizePoint(neighbor);
        neighbors.push_back(neighbor);
    }

    // Add neighbors in the XY plane
    Point neighbor(Point(point.x + _xy_resolution * cos(point.theta), point.y + _xy_resolution * sin(point.theta), point.theta));
    Point::normalizePoint(neighbor);

    // Check for collision before adding to neighbors
    if (!checkCollision(neighbor))
        neighbors.push_back(neighbor);

    return neighbors;
}

float AStar::calculateTravelCost(const Node &currentNode, const Node &neighborNode)
{
    float distance = Point::euclideanDistance(currentNode.point, neighborNode.point);
    float angleDifference = Point::absAngleDiff(currentNode.point.theta, neighborNode.point.theta);
    Point::normalizeTheta(angleDifference);
    return distance + (angleDifference / _theta_least_count);
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
    if (_map == nullptr)
    {
        return false; // No map set
    }

    if (checkCollision(_start_point) || checkCollision(_end_point))
    {
        std::cout << "Start or end point in collision" << std::endl;
        return false; // Start or end point is in collision
    }

    if (_start_point == _end_point)
    {
        path.push_back(_start_point);
        return true; // Start and end points are the same
    }

    reset();

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

        // std::cout << "Current Node - x: " << currentNode->point.x << " y : " << currentNode->point.y << " theta : " << currentNode->point.theta << std::endl;

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
            // std::cout << "Neighbour - x: " << neighborPoint.x << " y: " << neighborPoint.y << " theta: " << neighborPoint.theta << std::endl;
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

                // std::cout << "Cost g: " << temp_g << " h: " << temp_h << " f: " << temp_f << std::endl;

                openList.push(neighbor);
            }
        }
    }

    return false; // No path found
}

void AStar::reset()
{
    _node_data.clear();
    std::fill(_node_position.begin(), _node_position.end(), nullptr);
}

bool AStar::inTollerance(const Point &point)
{
    float dist = Point::euclideanDistance(point, _end_point);
    float theta_diff = Point::absAngleDiff(point.theta, _end_point.theta);

    if (dist <= _xy_tollerance && theta_diff <= _theta_tollerance)
    {
        return true;
    }

    return false;
}