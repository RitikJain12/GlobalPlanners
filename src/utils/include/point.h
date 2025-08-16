#pragma once

#include <math.h>

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

    // Equality operator for comparing two Point objects
    bool operator==(const Point &other) const;

    // Member variables
    float x;
    float y;
    float theta;

private:
    static float _threshold;       // Threshold for floating-point comparison
    static float _theta_threshold; // Threshold for theta comparison
};