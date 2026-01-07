#include "point.h"

Point::Point() {
  x = 0.0f;
  y = 0.0f;
  theta = 0.0f;
}

Point::Point(float x, float y, float theta) {
  this->x = x;
  this->y = y;
  this->theta = theta;
}

Point::Point(float x, float y, float theta, float steer) {
  this->x = x;
  this->y = y;
  this->theta = theta;
  this->steer = steer;
}

Point::Point(float x, float y, float theta, float steer, float linear_vel) {
  this->x = x;
  this->y = y;
  this->theta = theta;
  this->steer = steer;
  this->linear_vel = linear_vel;
  if (linear_vel < 0.0) this->reverse = true;
}

Point::Point(float x, float y) {
  this->x = x;
  this->y = y;
  this->theta = 0.0f;
}

Point::Point(const Point& other) {
  x = other.x;
  y = other.y;
  theta = other.theta;
  reverse = other.reverse;
  linear_vel = other.linear_vel;
}

bool Point::operator==(const Point& other) const {
  return (std::abs(x - other.x) < (_xy_least_count / 2.0)) &&
         (std::abs(y - other.y) < (_xy_least_count / 2.0)) &&
         (std::abs(theta - other.theta) < (_theta_least_count / 2.0));
}

void Point::operator=(const Point& other) {
  x = other.x;
  y = other.y;
  theta = other.theta;
  reverse = other.reverse;
  steer = other.steer;
  linear_vel = other.linear_vel;
}

void Point::setLeastCount(const float xy_least_count,
                          const float theta_least_count) {
  _xy_least_count = xy_least_count;
  _theta_least_count = theta_least_count;
}

float Point::euclideanDistance(const Point& p1, const Point& p2) {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

void Point::normalizeTheta(float& theta) {
  while (theta < 0 || theta >= (2 * M_PI)) {
    if (theta < 0)
      theta += (2 * M_PI);  // Normalize theta to be within [0, 2π]
    else if (theta >= (2 * M_PI))
      theta -= (2 * M_PI);
  }
}

void Point::normalizePoint(Point& point) {
  normalizeTheta(point.theta);
  point.x = std::round(point.x / _xy_least_count) * _xy_least_count;
  point.y = std::round(point.y / _xy_least_count) * _xy_least_count;
  point.theta =
      std::round(point.theta / _theta_least_count) * _theta_least_count;
}

float Point::slope(const Point& p1, const Point& p2) {
  return atan2((p2.y - p1.y), (p2.x - p1.x));
}

float Point::absAngleDiff(const float& p1, const float& p2) {
  return std::min((double)abs(p2 - p1), ((2 * M_PI) - abs(p2 - p1)));
}

float Point::_xy_least_count =
    0.01f;  // Default threshold for floating-point comparison
float Point::_theta_least_count =
    0.1f;  // Default threshold for theta comparison