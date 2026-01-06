#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "a_star.h"
#include "hybrid_a_star.h"
#include "map.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher(nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::Path path, geometry_msgs::msg::PolygonStamped footprint)
        : Node("map_publisher"), map_(map), path_(path), footprint_(footprint)
    {
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        footprint_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/footprint", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        map_publisher_->publish(map_);
        path_publisher_->publish(path_);
        footprint_publisher_->publish(footprint_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_publisher_;
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PolygonStamped footprint_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    bool use_astar = false;
    bool use_hybrid_astar = false;
    std::string mode = "astar";

    if (argc > 1)
    {
        mode = argv[1];
    }

    if (mode == "hybrid")
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using hybrid astar");
        use_hybrid_astar = true;
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Using astar");
        use_astar = true;
    }

    float map_width = 20;  // in meters
    float map_height = 20; // in meters
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = 0.1;
    map.info.width = static_cast<int>(ceil(map_width / map.info.resolution));   // in cells
    map.info.height = static_cast<int>(ceil(map_height / map.info.resolution)); // in cells

    std::shared_ptr<Map> map_ptr = std::make_shared<Map>(map_width, map_height, map.info.resolution);

    std::vector<Point>
        left_wall = {Point(0.0, 0.0), Point(0.0, 16.5), Point(0.1, 16.5), Point(0.1, 0.0)};
    map_ptr->setObstacles(left_wall);

    std::vector<Point>
        bottom_wall = {Point(0.0, 0.0), Point(2.6, 0.0), Point(2.6, 0.1), Point(0.0, 0.1)};
    map_ptr->setObstacles(bottom_wall);

    std::vector<Point>
        right_wall = {Point(2.6, 0.0), Point(2.6, 16.5), Point(2.7, 16.5), Point(2.7, 0.0)};
    map_ptr->setObstacles(right_wall);

    map.data = map_ptr->getMap();

    std::vector<Point>
        footprint = {Point(-1.0, 0.5), Point(-1.0, -0.5), Point(2.0, -0.5), Point(2.0, 0.5)};
    map_ptr->setFootprint(footprint);

    Point start;
    std::vector<Point> path_points;

    if (use_astar)
    {
        start = Point(1.0, 1.5, 1.5);
        AStar a_star(map_ptr, 8.0f);
        a_star.setStartPoint(start);
        a_star.setGoal(9.9f, 5.0f, 0.0f);

        if (!a_star.getPath(path_points))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No path found");
            exit(1);
        }
    }
    else if (use_hybrid_astar)
    {
        start = Point(0.9f, 1.5f, 1.57f);
        AStar *a_star = new HybridAStar(map_ptr, 0.3f, (2 * M_PI * 10));
        a_star->setStartPoint(start);
        a_star->setGoal(9.9f, 5.0f, 0.0f);
        // static_cast<HybridAStar *>(a_star)->resetObstacleHurestic();

        if (!a_star->getPath(path_points))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No path found");
            delete a_star;
            exit(1);
        }
        delete a_star;
    }

    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    for (const auto &point : path_points)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        pose.pose.orientation.z = sin(point.theta / 2.0);
        pose.pose.orientation.w = cos(point.theta / 2.0);
        std::cout << "Path point: (" << point.x << ", " << point.y << ", " << point.theta << ")" << std::endl;
        path.poses.push_back(pose);
    }

    std::vector<std::pair<float, float>> fp = map_ptr->getFootprint(start);
    geometry_msgs::msg::PolygonStamped ros_footprint;
    for (const std::pair<float, float> &p : fp)
    {
        geometry_msgs::msg::Point32 fp_point;
        fp_point.x = p.first;
        fp_point.y = p.second;
        ros_footprint.polygon.points.push_back(fp_point);
    }
    ros_footprint.header.frame_id = "map";

    rclcpp::spin(std::make_shared<MinimalPublisher>(map, path, ros_footprint));
    rclcpp::shutdown();
    return 0;
}