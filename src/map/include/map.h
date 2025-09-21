#pragma once

#include <math.h>
#include <vector>

class Map
{
private:
    float _width;
    float _height;
    float _resolution;
    int _width_index;
    int _height_index;
    std::vector<int> _map;

public:
    Map(const float &width, const float &height, const float &resolution);

    // bool getMaptoWorld(const int &index_x, const int &index_y, float &world_x, float &world_y);

    bool getWorldtoMap(int &index_x, int &index_y, const float &world_x, const float &world_y);

    int getIndex(const float &world_x, const float &world_y);

    int getCost(const int &index_x, const int &index_y);

    inline std::vector<int> getMap() { return _map; }

    inline void getMapDimentions(int &width, int &height, float &res)
    {
        width = _width_index;
        height = _height_index;
        res = _resolution;
    }
};
