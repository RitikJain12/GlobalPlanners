#include "a_star.h"

AStar::AStar(float xy_resolution, float theta_resolution)
    : startPoint(0.0f, 0.0f, 0.0f), endPoint(0.0f, 0.0f, 0.0f),
      _xy_resolution(xy_resolution), _theta_resolution(theta_resolution)
{
    float theta_threshold = M_PI / _theta_resolution; // Convert resolution to radians
    float xy_threshold = _xy_resolution / 2.0;        // Convert resolution to threshold
    Point::setThreshold(xy_threshold);
    Point::setThetaThreshold(theta_threshold);
}

void AStar::roundPointsToResolution(Point &point)
{
    point.x = std::round(point.x / _xy_resolution) * _xy_resolution;
    point.y = std::round(point.y / _xy_resolution) * _xy_resolution;
    point.theta = std::round(point.theta / _theta_resolution) * _theta_resolution;
}

void AStar::setStartPoint(const Point &start)
{
    startPoint = start;
    roundPointsToResolution(startPoint);
}

void AStar::setStartPoint(float x, float y, float theta)
{
    startPoint = Point(x, y, theta);
    roundPointsToResolution(startPoint);
}

void AStar::setEndPoint(const Point &end)
{
    endPoint = end;
    roundPointsToResolution(endPoint);
}

void AStar::setEndPoint(float x, float y, float theta)
{
    endPoint = Point(x, y, theta);
    roundPointsToResolution(endPoint);
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

std::vector<Point> AStar::getPath()
{
    // This method should implement the A* algorithm to compute the path
    // For now, we return an empty path as a placeholder
    return path;
}