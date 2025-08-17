#pragma once

#include "point.h"
#include <vector>

class AStar
{
public:
    // Constructor
    AStar(float xy_resolution = 1.0f, float theta_resolution = 8.0f);

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
    // Helper function to round points to the nearest resolution
    void roundPointsToResolution(Point &point);

    // Helper function to get neighbors of a point
    std::vector<Point> getNeighbors(const Point &point);

    // Member variables
    Point startPoint;        // Starting point of the path
    Point endPoint;          // Ending point of the path
    std::vector<Point> path; // Vector to store the path points

    float _xy_resolution;    // Resolution for XY coordinates
    float _theta_resolution; // Resolution for theta
};