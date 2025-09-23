#pragma once

#include <memory>
#include <vector>
#include <algorithm>
#include "map.h"

class DownsampledMap
{
private:
    std::shared_ptr<Map> _map;
    int _downsampling_factor;
    std::vector<int8_t> _downsampled_map;
    int _size_x;
    int _size_y;
    float _downsamled_resolution;
    int _downsampled_width_index;
    int _downsampled_height_index;

    void updateCostmap();

    void setCostOfCell(int x, int y);

    void setCost(int index_x, int index_y, int cost);

public:
    DownsampledMap(std::shared_ptr<Map> map, int downsampling_factor);

    inline int getSizeInX() { return _downsampled_width_index; }

    inline int getSizeInY() { return _downsampled_height_index; }
};
