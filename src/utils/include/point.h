#pragma once

#include <math.h>

#include <algorithm>

class Point {
 public:
  // Default constructor
  Point();

  // Parameterized constructor
  Point(float x, float y, float theta);

  Point(float x, float y, float theta, float steer);

  Point(float x, float y, float theta, float steer, float linear_vel);

  Point(float x, float y);

  // Copy constructor
  Point(const Point& other);

  // Set least count for x, y & theta
  static void setLeastCount(const float xy_least_count,
                            const float theta_least_count);

  // Calculate the Euclidean distance between two points
  static float euclideanDistance(const Point& p1, const Point& p2);

  // Round theta between 0 to 2pi
  static void normalizeTheta(float& theta);

  // calculate slope between 2 points
  static float slope(const Point& p1, const Point& p2);

  // calculate absolute diff between angle
  static float absAngleDiff(const float& p1, const float& p2);

  // Normalize point
  static void normalizePoint(Point& point);

  // Equality operator for comparing two Point objects
  bool operator==(const Point& other) const;

  // Assignment operator
  void operator=(const Point& other);

  // Member variables
  float x;
  float y;
  float theta;
  float steer = 0.0;
  bool reverse = false;
  float linear_vel = 0.0;

 private:
  static float _xy_least_count;     // Threshold for floating-point comparison
  static float _theta_least_count;  // Threshold for theta comparison
};