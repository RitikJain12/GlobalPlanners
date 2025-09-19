#include "a_star.h"
extern "C"
{
#include "dubins.h"
}

class HybridAStar : public AStar
{
public:
    HybridAStar(std::shared_ptr<Map> map, const float min_velocity = 0.3f, const float theta_resolution = 16.0f);

private:
    // Helper function to get neighbors of a point
    std::vector<Point> getNeighbors(const Point &point) override;

    float calculateTravelCost(const Node &currentNode, const Node &neighborNode) override;

    float calculateHeuristic(const Node &currentNode) override;

    float getDistanceHurestic(const Point &point);

    float getObstacleHurestic(const Point &point);

    bool _allow_reverse;
    float _min_velocity;
    float _wheelbase;
    float _steer_resolution;
    float _max_steer;
    int _steer_step;
    double _max_turnning_radius;
};
