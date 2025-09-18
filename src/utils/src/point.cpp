#include "point.h"

Point::Point()
{
    x = 0.0f;
    y = 0.0f;
    theta = 0.0f;
}

Point::Point(float x, float y, float theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;
}

Point::Point(float x, float y)
{
    this->x = x;
    this->y = y;
    this->theta = 0.0f;
}

Point::Point(const Point &other)
{
    x = other.x;
    y = other.y;
    theta = other.theta;
    reverse = other.reverse;
}

bool Point::operator==(const Point &other) const
{
    return (std::abs(x - other.x) < _threshold) &&
           (std::abs(y - other.y) < _threshold) &&
           (std::abs(theta - other.theta) < _theta_threshold);
}

void Point::operator=(const Point &other)
{
    x = other.x;
    y = other.y;
    theta = other.theta;
    reverse = other.reverse;
}

void Point::setThreshold(float newThreshold)
{
    _threshold = newThreshold;
}

void Point::setThetaThreshold(float newThetaThreshold)
{
    _theta_threshold = newThetaThreshold;
}

float Point::euclideanDistance(const Point &p1, const Point &p2)
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

void Point::roundTheta(float &theta)
{
    while (theta < 0 || theta >= (2 * M_PI))
    {
        if (theta < 0)
            theta += (2 * M_PI); // Normalize theta to be within [0, 2π]
        else if (theta >= (2 * M_PI))
            theta -= (2 * M_PI);
    }
}

float Point::slope(const Point &p1, const Point &p2)
{
    return atan2((p2.y - p1.y), (p2.x - p1.x));
}

float Point::absDiff(const float &p1, const float &p2)
{
    return std::min((double)abs(p2 - p1), ((2 * M_PI) - abs(p2 - p1)));
}

float Point::_threshold = 0.01f;      // Default threshold for floating-point comparison
float Point::_theta_threshold = 0.1f; // Default threshold for theta comparison