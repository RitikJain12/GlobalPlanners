#include "map.h"

Map::Map(const float& width, const float& height, const float& resolution)
    : _width(width), _height(height), _resolution(resolution) {
  _width_index = static_cast<int>(ceil(_width / _resolution));
  _height_index = static_cast<int>(ceil(_height / _resolution));
  _map.resize((_width_index * _height_index), 0);
}

bool Map::getWorldtoMap(int& index_x, int& index_y, const float& world_x,
                        const float& world_y) {
  if (world_x < 0 || world_x >= _width || world_y < 0 || world_y >= _height)
    return false;

  index_x = static_cast<int>(std::round(world_x / _resolution));
  index_y = static_cast<int>(std::round(world_y / _resolution));

  return true;
}

int Map::getIndex(const float& world_x, const float& world_y) {
  int index_x;
  int index_y;
  getWorldtoMap(index_x, index_y, world_x, world_y);
  return (index_x + (index_y * _width_index));
}

int8_t Map::getCost(const int& index_x, const int& index_y) {
  int index = index_x + (index_y * _width_index);

  if (index < (_width_index * _height_index)) return _map[index];

  return 1;
}

void Map::setFootprint(const std::vector<Point> footprint) {
  _footprint = footprint;
}

void Map::setCost(int index_x, int index_y, int cost) {
  int index = index_x + (index_y * _width_index);

  if (index < (_width_index * _height_index)) _map[index] = cost;
}

void Map::getLineCells(int x0, int x1, int y0, int y1,
                       std::vector<std::pair<int, int>>& pts) {
  // Bresenham Ray-Tracing
  int deltax = abs(x1 - x0);  // The difference between the x's
  int deltay = abs(y1 - y0);  // The difference between the y's
  int x = x0;                 // Start x off at the first pixel
  int y = y0;                 // Start y off at the first pixel

  int xinc1, xinc2, yinc1, yinc2;
  int den, num, numadd, numpixels;

  std::pair<int, int> pt;

  if (x1 >= x0)  // The x-values are increasing
  {
    xinc1 = 1;
    xinc2 = 1;
  } else  // The x-values are decreasing
  {
    xinc1 = -1;
    xinc2 = -1;
  }

  if (y1 >= y0)  // The y-values are increasing
  {
    yinc1 = 1;
    yinc2 = 1;
  } else  // The y-values are decreasing
  {
    yinc1 = -1;
    yinc2 = -1;
  }

  if (deltax >= deltay)  // There is at least one x-value for every y-value
  {
    xinc1 = 0;  // Don't change the x when numerator >= denominator
    yinc2 = 0;  // Don't change the y for every iteration
    den = deltax;
    num = deltax / 2;
    numadd = deltay;
    numpixels = deltax;  // There are more x-values than y-values
  } else                 // There is at least one y-value for every x-value
  {
    xinc2 = 0;  // Don't change the x for every iteration
    yinc1 = 0;  // Don't change the y when numerator >= denominator
    den = deltay;
    num = deltay / 2;
    numadd = deltax;
    numpixels = deltay;  // There are more y-values than x-values
  }

  for (int curpixel = 0; curpixel <= numpixels; curpixel++) {
    pt.first = x;  // Draw the current pixel
    pt.second = y;
    pts.push_back(pt);

    num += numadd;   // Increase the numerator by the top of the fraction
    if (num >= den)  // Check if numerator >= denominator
    {
      num -= den;  // Calculate the new numerator value
      x += xinc1;  // Change the x as appropriate
      y += yinc1;  // Change the y as appropriate
    }
    x += xinc2;  // Change the x as appropriate
    y += yinc2;  // Change the y as appropriate
  }
}

std::vector<std::pair<int, int>> Map::getFootprintCells(const Point& point) {
  std::vector<std::pair<int, int>> footprint_cells;
  double cos_theta = std::cos(point.theta);
  double sin_theta = std::sin(point.theta);

  int n = _footprint.size();
  for (int i = 0; i < n; i++) {
    Point p1 = _footprint[i];
    int x1 = static_cast<int>((cos_theta * p1.x - sin_theta * p1.y + point.x) /
                              _resolution);
    int y1 = static_cast<int>((sin_theta * p1.x + cos_theta * p1.y + point.y) /
                              _resolution);

    Point p2 = _footprint[(i + 1) % n];
    int x2 = static_cast<int>((cos_theta * p2.x - sin_theta * p2.y + point.x) /
                              _resolution);
    int y2 = static_cast<int>((sin_theta * p2.x + cos_theta * p2.y + point.y) /
                              _resolution);

    getLineCells(x1, x2, y1, y2, footprint_cells);
  }

  return footprint_cells;
}

std::vector<std::pair<float, float>> Map::getFootprint(const Point& point) {
  std::vector<std::pair<float, float>> footprint;
  double cos_theta = std::cos(point.theta);
  double sin_theta = std::sin(point.theta);

  int n = _footprint.size();
  for (int i = 0; i < n; i++) {
    Point p1 = _footprint[i];
    float x1 = (cos_theta * p1.x - sin_theta * p1.y + point.x);
    float y1 = (sin_theta * p1.x + cos_theta * p1.y + point.y);

    footprint.push_back(std::make_pair(x1, y1));
  }

  return footprint;
}

void Map::setObstacles(std::vector<Point> polygon) {
  std::vector<std::pair<int, int>> footprint_cells;

  int n = polygon.size();
  for (int i = 0; i < n; i++) {
    Point p1 = polygon[i];
    int x1 = static_cast<int>(p1.x / _resolution);
    int y1 = static_cast<int>(p1.y / _resolution);

    Point p2 = polygon[(i + 1) % n];
    int x2 = static_cast<int>(p2.x / _resolution);
    int y2 = static_cast<int>(p2.y / _resolution);

    getLineCells(x1, x2, y1, y2, footprint_cells);
  }

  for (const std::pair<int, int>& p : footprint_cells) {
    setCost(p.first, p.second, 254);
  }
}