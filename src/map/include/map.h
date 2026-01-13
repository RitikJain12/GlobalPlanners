#pragma once

#include <math.h>

#include <vector>

#include "point.h"

class Map {
 private:
  float _width;
  float _height;
  float _resolution;
  int _width_index;
  int _height_index;
  std::vector<int8_t> _map;
  std::vector<Point> _footprint;

  void getLineCells(int x0, int x1, int y0, int y1,
                    std::vector<std::pair<int, int>>& pts);

  void setCost(int index_x, int index_y, int8_t cost);

 public:
  Map(const float& width, const float& height, const float& resolution);

  bool getWorldtoMap(int& index_x, int& index_y, const float& world_x,
                     const float& world_y);

  int getIndex(const float& world_x, const float& world_y);

  int8_t getCost(const int& index_x, const int& index_y);

  int8_t getCost(const unsigned int index);

  // Method to set the footprint
  void setFootprint(const std::vector<Point> footprint);

  inline std::vector<int8_t> getMap() { return _map; }

  inline void getMapDimentions(int& width, int& height, float& res) {
    width = _width_index;
    height = _height_index;
    res = _resolution;
  }

  inline int getSizeInX() { return _width_index; }

  inline int getSizeInY() { return _height_index; }

  inline float getResolution() { return _resolution; }

  // Transform footprint coordinates and return map cells
  std::vector<std::pair<int, int>> getFootprintCells(const Point& point);

  // Transform footprint coordinates
  std::vector<std::pair<float, float>> getFootprint(const Point& point);

  // set obstacles
  void setObstacles(std::vector<Point> polygon);
};
