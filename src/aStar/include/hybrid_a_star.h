#include "a_star.h"

class HybridAStar : public AStar
{
public:
    HybridAStar(const float xy_resolution = 0.1f, const float theta_resolution = 8.0f);

private:
    // Helper function to get neighbors of a point
    std::vector<Point> getNeighbors(const Point &point);

    bool _allow_reverse;
    float _min_velocity;
    float _wheelbase;
};
