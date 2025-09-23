#include "downsampled_map.h"

DownsampledMap::DownsampledMap(std::shared_ptr<Map> map,
                               int downsampling_factor) : _map(map),
                                                          _downsampling_factor(downsampling_factor)
{
    float res;
    _map->getMapDimentions(_size_x, _size_y, res);
    _downsamled_resolution = res * _downsampling_factor;
    _downsampled_width_index = ceil((static_cast<float>(_size_x * res) / _downsampling_factor) / _downsamled_resolution);
    _downsampled_height_index = ceil((static_cast<float>(_size_y * res) / _downsampling_factor) / _downsamled_resolution);
    _downsampled_map.resize((_downsampled_width_index * _downsampled_height_index), 0);
    updateCostmap();
}

void DownsampledMap::updateCostmap()
{
    for (int i = 0; i < _downsampled_width_index; i++)
    {
        for (int j = 0; j < _downsampled_height_index; j++)
        {
            setCostOfCell(i, j);
        }
    }
}

void DownsampledMap::setCostOfCell(int x, int y)
{
    int mx, my;
    int8_t cost = 0;
    int x_offset = x * _downsampling_factor;
    int y_offset = y * _downsampling_factor;

    for (int i = 0; i < _downsampling_factor; ++i)
    {
        mx = x_offset + i;
        if (mx >= _size_x)
        {
            continue;
        }
        for (int j = 0; j < _downsampling_factor; ++j)
        {
            my = y_offset + j;
            if (my >= _size_y)
            {
                continue;
            }
            cost = std::max(cost, _map->getCost(mx, my));
        }
    }

    setCost(x, y, cost);
}

void DownsampledMap::setCost(int index_x, int index_y, int cost)
{
    int index = index_x + (index_y * _downsampled_width_index);

    if (index < (_downsampled_width_index * _downsampled_height_index))
        _downsampled_map[index] = cost;
}