#include "map.h"

Map::Map(const float &width, const float &height, const float &resolution)
    : _width(width), _height(height), _resolution(resolution)
{
    _width_index = static_cast<int>(ceil(_width * _resolution));
    _height_index = static_cast<int>(ceil(_height * _resolution));
    _map.resize((_width_index * _height_index), 0);
}

void Map::getMaptoWorld(const int &index_x, const int &index_y, float &world_x, float &world_y)
{
}

void Map::getWorldtoMap(int &index_x, int &index_y, const float &world_x, const float &world_y)
{
    index_x = static_cast<int>(std::round(world_x / _resolution));
    index_y = static_cast<int>(std::round(world_y / _resolution));
}

float Map::getIndex(const float &world_x, const float &world_y)
{
    int index_x;
    int index_y;
    getWorldtoMap(index_x, index_y, world_x, world_y);
    return (index_x + (index_y * _width_index));
}

int Map::getCost(const int &index_x, const int &index_y)
{
    int index = index_x + (index_y * _width_index);

    if (index < (_width_index * _height_index))
        return _map[index];

    return 1;
}