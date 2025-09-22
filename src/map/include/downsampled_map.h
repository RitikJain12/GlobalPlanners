#pragma once

#include <memory>
#include <vector>
#include "map.h"

class DownsampledMap
{
private:
    std::shared_ptr<Map> _map;
    int _downsampling_factor;
    std::vector<int8_t> _map;
    float _resolution;
    int _downsampled_width_index;
    int _downsampled_height_index;

public:
    DownsampledMap(std::shared_ptr<Map> map, int downsampling_factor);
};
