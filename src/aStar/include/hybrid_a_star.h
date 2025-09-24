#include "a_star.h"
#include "downsampled_map.h"
extern "C"
{
#include "dubins.h"
}

class HybridAStar : public AStar
{
public:
    HybridAStar(std::shared_ptr<Map> map, const float min_velocity = 0.3f, const float theta_resolution = 16.0f);

    void resetObstacleHurestic();

private:
    // Helper function to get neighbors of a point
    std::vector<Point> getNeighbors(const Point &point) override;

    std::vector<Point> discretizeVel(const Point &point);

    std::vector<Point> discretizeAcc(const Point &point);

    float calculateTravelCost(const Node &currentNode, const Node &neighborNode) override;

    float calculateHeuristic(const Node &currentNode) override;

    float getDistanceHurestic(const Point &point);

    float getObstacleHurestic(const Point &point);

    float DistanceHeuristic(int curr_index, int width, int goal_x, int goal_y);

    bool _use_dynamic;
    bool _allow_reverse;
    float _min_velocity;
    float _max_velocity;
    float _max_reverse_vel;
    float _min_linear_acc;
    float _wheelbase;
    float _steer_resolution;
    float _max_steer;
    int _steer_step;
    double _max_turnning_radius;
    DownsampledMap _downsampled_map;
    std::vector<float> _obstacle_heuristic_map;
    std::vector<std::pair<float, int>> _obstacle_heuristic_queue;
};

struct ObstacleHeuristicComparator
{
    bool operator()(const std::pair<float, int> &a, const std::pair<float, int> &b) const
    {
        return a.first > b.first;
    }
};