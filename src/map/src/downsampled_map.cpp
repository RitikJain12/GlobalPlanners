#include "downsampled_map.h"

DownsampledMap::DownsampledMap(std::shared_ptr<Map> map,
                               int downsampling_factor) : _map(map),
                                                          _downsampling_factor(downsampling_factor)
{
}