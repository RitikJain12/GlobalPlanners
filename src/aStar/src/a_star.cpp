#include "a_star.h"

AStar::AStar(std::shared_ptr<Map> map, const float theta_resolution)
    : _start_point(Point()), _end_point(Point()),
      _theta_resolution(theta_resolution),
      _map(map)
{
    _xy_tollerance = 0.1;
    _theta_tollerance = 0.1;

    _theta_least_count = (2 * M_PI) / _theta_resolution; // Convert resolution to radians

    int grid_width;
    int grid_height;

    _map->getMapDimentions(grid_width, grid_height, _xy_resolution);

    _node_data.reserve((unsigned long int)grid_width * grid_height * ceil(_theta_resolution));             // Reserve space for nodes
    _node_position.resize((unsigned long int)grid_width * grid_height * ceil(_theta_resolution), nullptr); // Initialize node pointers

    Point::setLeastCount(_xy_resolution, _theta_least_count); // Set least count for Point class
}

void AStar::setStartPoint(const Point &start)
{
    _start_point = Point(start);
}

void AStar::setStartPoint(float x, float y, float theta)
{
    _start_point = Point(x, y, theta);
}

void AStar::setGoal(const Point &end)
{
    _end_point = Point(end);
}

void AStar::setGoal(float x, float y, float theta)
{
    _end_point = Point(x, y, theta);
}

void AStar::setFootprint(const std::vector<Point> footprint)
{
    _footprint = footprint;
}

void AStar::setNodeAtPose(Point point, Node *node)
{
    // roundPointsToResolution(point);
    int index_xy = _map->getIndex(point.x, point.y);
    unsigned long int index = (point.theta / _theta_least_count) + (index_xy * _theta_resolution);
    _node_position[index] = node;
}

Node *AStar::getNodeAtPose(Point point)
{
    // roundPointsToResolution(point);
    int index_xy = _map->getIndex(point.x, point.y);
    unsigned long int index = (point.theta / _theta_least_count) + (index_xy * _theta_resolution);
    return _node_position[index];
}

void AStar::getLineCells(int x0, int x1, int y0, int y1, std::vector<std::pair<int, int>> &pts)
{
    // Bresenham Ray-Tracing
    int deltax = abs(x1 - x0); // The difference between the x's
    int deltay = abs(y1 - y0); // The difference between the y's
    int x = x0;                // Start x off at the first pixel
    int y = y0;                // Start y off at the first pixel

    int xinc1, xinc2, yinc1, yinc2;
    int den, num, numadd, numpixels;

    std::pair<int, int> pt;

    if (x1 >= x0) // The x-values are increasing
    {
        xinc1 = 1;
        xinc2 = 1;
    }
    else // The x-values are decreasing
    {
        xinc1 = -1;
        xinc2 = -1;
    }

    if (y1 >= y0) // The y-values are increasing
    {
        yinc1 = 1;
        yinc2 = 1;
    }
    else // The y-values are decreasing
    {
        yinc1 = -1;
        yinc2 = -1;
    }

    if (deltax >= deltay) // There is at least one x-value for every y-value
    {
        xinc1 = 0; // Don't change the x when numerator >= denominator
        yinc2 = 0; // Don't change the y for every iteration
        den = deltax;
        num = deltax / 2;
        numadd = deltay;
        numpixels = deltax; // There are more x-values than y-values
    }
    else // There is at least one y-value for every x-value
    {
        xinc2 = 0; // Don't change the x for every iteration
        yinc1 = 0; // Don't change the y when numerator >= denominator
        den = deltay;
        num = deltay / 2;
        numadd = deltax;
        numpixels = deltay; // There are more y-values than x-values
    }

    for (int curpixel = 0; curpixel <= numpixels; curpixel++)
    {
        pt.first = x; // Draw the current pixel
        pt.second = y;
        pts.push_back(pt);

        num += numadd;  // Increase the numerator by the top of the fraction
        if (num >= den) // Check if numerator >= denominator
        {
            num -= den; // Calculate the new numerator value
            x += xinc1; // Change the x as appropriate
            y += yinc1; // Change the y as appropriate
        }
        x += xinc2; // Change the x as appropriate
        y += yinc2; // Change the y as appropriate
    }
}

std::vector<std::pair<int, int>> AStar::getFootprintCells(const Point &point)
{
    std::vector<std::pair<int, int>> footprint_cells;
    double cos_theta = std::cos(point.theta);
    double sin_theta = std::sin(point.theta);

    int n = _footprint.size();
    for (int i = 0; i < n; i++)
    {
        Point p1 = _footprint[i];
        int x1 = static_cast<int>((cos_theta * p1.x - sin_theta * p1.y + point.x) / _xy_resolution);
        int y1 = static_cast<int>((sin_theta * p1.x + cos_theta * p1.y + point.y) / _xy_resolution);

        Point p2 = _footprint[(i + 1) % _footprint.size()];
        int x2 = static_cast<int>((cos_theta * p2.x - sin_theta * p2.y + point.x) / _xy_resolution);
        int y2 = static_cast<int>((sin_theta * p2.x + cos_theta * p2.y + point.y) / _xy_resolution);

        getLineCells(x1, x2, y1, y2, footprint_cells);
    }

    return footprint_cells;
}

bool AStar::checkCollision(Point point)
{
    int index_x, index_y;
    if (!_map->getWorldtoMap(index_x, index_y, point.x, point.y))
        return true;
    if (_map->getCost(index_x, index_y) != 0)
        return true;

    std::vector<std::pair<int, int>> fp_points = getFootprintCells(point);

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