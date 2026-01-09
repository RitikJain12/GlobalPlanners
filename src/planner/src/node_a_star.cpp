#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "a_star.h"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "hybrid_a_star.h"
#include "map.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

class PlannerAStar : public rclcpp::Node {
 public:
  PlannerAStar() : Node("astar_planner") {
    map_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    footprint_publisher_ =
        this->create_publisher<geometry_msgs::msg::PolygonStamped>("/footprint",
                                                                   10);

    viz_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/planner_visualization", 10);

    this->declare_parameter("planner.type", "AStar");
    this->declare_parameter("planner.debug_visualization", false);

    this->declare_parameter("planner.theta_resolution", 8.0);
    this->declare_parameter("planner.xy_tolerance", 0.1);
    this->declare_parameter("planner.theta_tolerance", 0.1);
    this->declare_parameter("planner.timeout", 20.0);

    this->declare_parameter("planner.min_velocity", 0.3);
    this->declare_parameter("planner.use_dynamic", false);
    this->declare_parameter("planner.allow_reverse", false);
    this->declare_parameter("planner.wheelbase", 2.0);
    this->declare_parameter("planner.max_velocity", 0.7);
    this->declare_parameter("planner.min_linear_acc", 0.1);
    this->declare_parameter("planner.steer_resolution", 5.625);
    this->declare_parameter("planner.max_steer", 33.75);

    this->declare_parameter("map.width", 20.0);
    this->declare_parameter("map.height", 20.0);
    this->declare_parameter("map.resolution", 0.1);

    initialize_env();

    initialize_planner();

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1,
        std::bind(&PlannerAStar::goalCallback, this, std::placeholders::_1));

    start_sub_ = this->create_subscription<
        geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1,
        std::bind(&PlannerAStar::startCallback, this, std::placeholders::_1));

    obstacle_sub_ =
        this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/add_polygon", 1,
            std::bind(&PlannerAStar::obstacleCallback, this,
                      std::placeholders::_1));

    timer_ = this->create_wall_timer(
        500ms, std::bind(&PlannerAStar::timer_callback, this));

    this->get_parameter("planner.debug_visualization", debug_visualization_);
    if (debug_visualization_) {
      a_star_->setVisualizationCallback(
          std::bind(&PlannerAStar::visvualizer, this, std::placeholders::_1));
    }
  }

  ~PlannerAStar() { delete a_star_; }

 private:
  Point getPoint(const geometry_msgs::msg::Pose pose_msg) {
    return Point(pose_msg.position.x, pose_msg.position.y,
                 2 * atan2(pose_msg.orientation.z, pose_msg.orientation.w));
  }

  void initialize_env() {
    float map_width;   // in meters
    float map_height;  // in meters
    float resoluion;

    this->get_parameter("map.width", map_width);
    this->get_parameter("map.height", map_height);
    this->get_parameter("map.resolution", resoluion);

    nav_msgs::msg::OccupancyGrid map;
    map_.header.frame_id = "map";
    map_.info.resolution = resoluion;
    map_.info.width =
        static_cast<int>(ceil(map_width / map_.info.resolution));  // in cells
    map_.info.height =
        static_cast<int>(ceil(map_height / map_.info.resolution));  // in cells

    map_ptr_ =
        std::make_shared<Map>(map_width, map_height, map_.info.resolution);

    map_.data = map_ptr_->getMap();

    std::vector<Point> footprint = {Point(-1.0, 0.5), Point(-1.0, -0.5),
                                    Point(2.0, -0.5), Point(2.0, 0.5)};
    map_ptr_->setFootprint(footprint);
  }

  void set_footprint(Point start) {
    std::vector<std::pair<float, float>> fp = map_ptr_->getFootprint(start);

    footprint_.polygon.points.clear();
    for (const std::pair<float, float>& p : fp) {
      geometry_msgs::msg::Point32 fp_point;
      fp_point.x = p.first;
      fp_point.y = p.second;
      footprint_.polygon.points.push_back(fp_point);
    }
    footprint_.header.frame_id = "map";
  }

  void initialize_planner() {
    float theta_resolution;
    float xy_tolerance;
    float theta_tolerance;
    float timeout;

    if (!this->get_parameter("planner.theta_resolution", theta_resolution)) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Defaulting theta resolution to 8.0");
      theta_resolution = 8.0;
    }

    if (!this->get_parameter("planner.xy_tolerance", xy_tolerance)) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Defaulting distance tolerance to 0.1");
      xy_tolerance = 0.1;
    }

    if (!this->get_parameter("planner.theta_tolerance", theta_tolerance)) {
      RCLCPP_WARN(rclcpp::get_logger("rclcpp"),
                  "Defaulting theta tolerance to 0.1");
      theta_tolerance = 0.1;
    }
    this->get_parameter("planner.timeout", timeout);

    std::string planner_type;
    this->get_parameter("planner.type", planner_type);

    if (planner_type == "HybridAStar") {
      float min_velocity;
      bool use_dynamic;
      bool allow_reverse;
      float wheelbase;
      float max_velocity;
      float min_linear_acc;
      float steer_resolution;
      float max_steer;

      this->get_parameter("planner.min_velocity", min_velocity);
      this->get_parameter("planner.use_dynamic", use_dynamic);
      this->get_parameter("planner.allow_reverse", allow_reverse);
      this->get_parameter("planner.wheelbase", wheelbase);
      this->get_parameter("planner.max_velocity", max_velocity);
      this->get_parameter("planner.min_linear_acc", min_linear_acc);
      this->get_parameter("planner.steer_resolution", steer_resolution);
      this->get_parameter("planner.max_steer", max_steer);

      a_star_ = new HybridAStar(
          map_ptr_, min_velocity, theta_resolution, xy_tolerance,
          theta_tolerance, timeout, use_dynamic, allow_reverse, wheelbase,
          max_velocity, min_linear_acc, steer_resolution, max_steer);
    } else {
      a_star_ = new AStar(map_ptr_, theta_resolution, xy_tolerance,
                          theta_tolerance, timeout);
    }

    Point start = Point(1.0, 1.5, 1.57);
    a_star_->setStartPoint(start);

    set_footprint(start);
  }

  void try_plan() {
    viz_markers_ = visualization_msgs::msg::MarkerArray();
    rclcpp::Time start_time = this->now();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Planning path...");
    std::vector<Point> path_points;
    path_ = nav_msgs::msg::Path();
    path_.header.frame_id = "map";

    if (!a_star_->getPath(path_points)) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No path found");
      return;
    }

    for (const auto& point : path_points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.orientation.z = sin(point.theta / 2.0);
      pose.pose.orientation.w = cos(point.theta / 2.0);
      // std::cout << "Path point: (" << point.x << ", " << point.y << ", " <<
      // point.theta << ")" << std::endl;
      path_.poses.push_back(pose);
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Path found with %zu points in %.3f seconds",
                path_points.size(), (this->now() - start_time).seconds());
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped& msg) {
    a_star_->setGoal(getPoint(msg.pose));
    try_plan();
  }

  void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped& msg) {
    Point start = Point(getPoint(msg.pose.pose));
    a_star_->setStartPoint(start);
    set_footprint(start);
  }

  void obstacleCallback(const geometry_msgs::msg::PolygonStamped& msg) {
    std::vector<Point> obstacle;
    for (geometry_msgs::msg::Point32 point : msg.polygon.points) {
      obstacle.push_back(Point(point.x, point.y));
    }
    map_ptr_->setObstacles(obstacle);
    map_.data = map_ptr_->getMap();
  }

  void timer_callback() {
    map_publisher_->publish(map_);
    path_publisher_->publish(path_);
    footprint_publisher_->publish(footprint_);
  }

  void visvualizer(const AStar::VisvualizationData& data) {
    if (viz_markers_.markers.empty()) {
      visualization_msgs::msg::Marker open_set_marker;
      open_set_marker.header.frame_id = "map";
      open_set_marker.ns = "open_set";
      open_set_marker.id = 0;
      open_set_marker.type = visualization_msgs::msg::Marker::POINTS;
      open_set_marker.action = visualization_msgs::msg::Marker::ADD;
      open_set_marker.scale.x = 0.1;
      open_set_marker.scale.y = 0.1;
      open_set_marker.color.a = 1.0;
      open_set_marker.color.r = 0.0;
      open_set_marker.color.g = 1.0;
      open_set_marker.color.b = 0.0;

      visualization_msgs::msg::Marker closed_set_marker;
      closed_set_marker.header.frame_id = "map";
      closed_set_marker.ns = "closed_set";
      closed_set_marker.id = 1;
      closed_set_marker.type = visualization_msgs::msg::Marker::POINTS;
      closed_set_marker.action = visualization_msgs::msg::Marker::ADD;
      closed_set_marker.scale.x = 0.1;
      closed_set_marker.scale.y = 0.1;
      closed_set_marker.color.a = 1.0;
      closed_set_marker.color.r = 1.0;
      closed_set_marker.color.g = 0.0;
      closed_set_marker.color.b = 0.0;

      visualization_msgs::msg::Marker current_marker;
      current_marker.header.frame_id = "map";
      current_marker.ns = "current_node";
      current_marker.id = 2;
      current_marker.type = visualization_msgs::msg::Marker::ARROW;
      current_marker.action = visualization_msgs::msg::Marker::ADD;
      current_marker.scale.x = 0.2;
      current_marker.scale.y = 0.2;
      current_marker.scale.z = 0.1;
      current_marker.color.a = 1.0;
      current_marker.color.r = 0.0;
      current_marker.color.g = 0.0;
      current_marker.color.b = 1.0;

      viz_markers_.markers.push_back(open_set_marker);
      viz_markers_.markers.push_back(closed_set_marker);
      viz_markers_.markers.push_back(current_marker);
    }

    for (const Point& p : data.neighbors) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = 0.0;
      viz_markers_.markers[0].points.push_back(point);
    }

    geometry_msgs::msg::Point point;
    point.x = data.current.x;
    point.y = data.current.y;
    point.z = 0.0;
    viz_markers_.markers[1].points.push_back(point);

    viz_markers_.markers[2].pose.position.x = data.current.x;
    viz_markers_.markers[2].pose.position.y = data.current.y;
    viz_markers_.markers[2].pose.position.z = 0.0;
    viz_markers_.markers[2].pose.orientation.z = sin(data.current.theta / 2.0);
    viz_markers_.markers[2].pose.orientation.w = cos(data.current.theta / 2.0);

    viz_publisher_->publish(viz_markers_);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr
      footprint_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      viz_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr
      obstacle_sub_;

  nav_msgs::msg::OccupancyGrid map_;
  nav_msgs::msg::Path path_;
  geometry_msgs::msg::PolygonStamped footprint_;
  visualization_msgs::msg::MarkerArray viz_markers_;

  std::shared_ptr<Map> map_ptr_;
  AStar* a_star_;

  bool debug_visualization_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PlannerAStar>());
  rclcpp::shutdown();
  return 0;
}