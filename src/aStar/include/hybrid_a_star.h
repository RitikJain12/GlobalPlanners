#include "a_star.h"
extern "C" {
#include "dubins.h"
}

class HybridAStar : public AStar {
 public:
  HybridAStar(std::shared_ptr<Map> map, const float min_velocity = 0.3f,
              const float theta_resolution = 16.0f,
              const float xy_tolerance = 0.1f,
              const float theta_tolerance = 0.1f, const float timeout = 20.0f,
              const bool use_dynamic = false, const bool allow_reverse = false,
              const float wheelbase = 2.0f, const float max_velocity = 0.7f,
              const float min_linear_acc = 0.1f,
              const float steer_resolution = 5.625f,
              const float max_steer = 33.75f);

 private:
  void resetObstacleHurestic();

  void resetDistanceHurestic();

  // Helper function to get neighbors of a point
  std::vector<Point> getNeighbors(const Point& point) override;

  std::vector<Point> discretizeVel(const Point& point);

  std::vector<Point> discretizeAcc(const Point& point);

  float calculateTravelCost(const Node& currentNode,
                            const Node& neighborNode) override;

  float calculateHeuristic(const Node& currentNode) override;

  float getDistanceHurestic(const Point& point);

  float getObstacleHurestic(const Point& point);

  float DistanceHeuristic(int curr_index, int width, int goal_x, int goal_y);

  void reset() override;

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
  double _min_turnning_radius;

  std::vector<float> _distance_heuristic_map;

  std::vector<float> _obstacle_heuristic_map;
  std::vector<std::pair<float, int>> _obstacle_heuristic_queue;

  float _turn_penalty = 1.2f;
  float _change_penalty = 0.7f;
  float _reverse_penalty = 2.0f;
};

struct ObstacleHeuristicComparator {
  bool operator()(const std::pair<float, int>& a,
                  const std::pair<float, int>& b) const {
    return a.first > b.first;
  }
};