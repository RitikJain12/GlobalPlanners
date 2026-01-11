#include "downsampled_map.h"

DownsampledMap::DownsampledMap(std::shared_ptr<Map> map,
                               int downsampling_factor)
    : _map(map), _downsampling_factor(downsampling_factor) {
  float res;
  _map->getMapDimentions(_size_x, _size_y, res);
  _downsamled_resolution = res * _downsampling_factor;
  _downsampled_width_index =
      ceil((static_cast<float>(_size_x * res) / _downsampling_factor) /
           _downsamled_resolution);
  _downsampled_height_index =
      ceil((static_cast<float>(_size_y * res) / _downsampling_factor) /
           _downsamled_resolution);
  _downsampled_map.resize(
      (_downsampled_width_index * _downsampled_height_index), 0);
  updateCostmap();
}

void DownsampledMap::updateCostmap() {
  for (int i = 0; i < _downsampled_width_index; i++) {
    for (int j = 0; j < _downsampled_height_index; j++) {
      setCostOfCell(i, j);
    }
  }
}

void DownsampledMap::setCostOfCell(int x, int y) {
  int mx, my;
  int8_t cost = 0;
  int x_offset = x * _downsampling_factor;
  int y_offset = y * _downsampling_factor;

  for (int i = 0; i < _downsampling_factor; ++i) {
    mx = x_offset + i;
    if (mx >= _size_x) {
      continue;
    }
    for (int j = 0; j < _downsampling_factor; ++j) {
      my = y_offset + j;
      if (my >= _size_y) {
        continue;
      }
      cost = std::max(cost, _map->getCost(mx, my));
    }
  }

  setCost(x, y, cost);
}

void DownsampledMap::setCost(int index_x, int index_y, int cost) {
  int index = index_x + (index_y * _downsampled_width_index);

  if (index < (_downsampled_width_index * _downsampled_height_index))
    _downsampled_map[index] = cost;
}

int8_t DownsampledMap::getCost(int index) {
  if (index < (_downsampled_width_index * _downsampled_height_index)) {
    return _downsampled_map[index];
  }

  return 254;
}

bool DownsampledMap::getWorldtoMap(int& index_x, int& index_y, float world_x,
                                   float world_y) {
  world_x = ceil(world_x / _downsampling_factor);
  world_y = ceil(world_y / _downsampling_factor);

  if (world_x < 0 || world_x >= _size_x || world_y < 0 || world_y >= _size_y)
    return false;

  index_x = static_cast<int>(std::round(world_x / _downsamled_resolution));
  index_y = static_cast<int>(std::round(world_y / _downsamled_resolution));

  return true;
}

int DownsampledMap::getIndex(const float& world_x, const float& world_y) {
  int index_x;
  int index_y;
  getWorldtoMap(index_x, index_y, world_x, world_y);
  return (index_x + (index_y * _downsampled_width_index));
}