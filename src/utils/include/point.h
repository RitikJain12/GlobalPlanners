#pragma once

#include <math.h>
#include <algorithm>

class Point
{
public:
    // Default constructor
    Point();

    // Parameterized constructor
    Point(float x, float y, float theta);

    // Copy constructor
    Point(const Point &other);

    // Set a new threshold for floating-point comparison
    static void setThreshold(float newThreshold);

    // Set a new threshold for theta comparison
    static void setThetaThreshold(float newThetaThreshold);

    // Calculate the Euclidean distance between two points
    static float euclideanDistance(const Point &p1, const Point &p2);

    // Round theta between 0 to 2pi
    static void roundTheta(float &theta);

    // calculate slope between 2 points
    static float slope(const Point &p1, const Point &p2);

    // calculate absolute diff between angle
    static float absDiff(const float &p1, const float &p2);

    // Equality operator for comparing two Point objects
    bool operator==(const Point &other) const;

    // Assignment operator
    void operator=(const Point &other);

    // Member variables
    float x;
    float y;
    float theta;

private:
    static float _threshold;       // Threshold for floating-point comparison
    static float _theta_threshold; // Threshold for theta comparison
};