#pragma once

#include "point.h"
#include <vector>

class AStar
{
public:
    // Constructor
    AStar(float xy_threshold = 0.01f, float theta_threshold = 0.1f);

    // Method to set the start point
    void setStartPoint(const Point &start);

    // Method to set the start point with coordinates
    void setStartPoint(float x, float y, float theta);

    // Method to set the end point
    void setEndPoint(const Point &end);

    // Method to set the end point with coordinates
    void setEndPoint(float x, float y, float theta);

    // Method to get the path
    std::vector<Point> getPath();

private:
    Point startPoint; // Starting point of the path
    Point endPoint;   // Ending point of the path
    std::vector<Point> path; // Vector to store the path points
};