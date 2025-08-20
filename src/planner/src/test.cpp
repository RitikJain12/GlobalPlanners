#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "a_star.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher(nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::Path path)
        : Node("map_publisher"), map_(map), path_(path)
    {
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        map_publisher_->publish(map_);
        path_publisher_->publish(path_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = 1.0;
    map.info.width = 10;
    map.info.height = 10;
    map.data.resize(map.info.width * map.info.height, 0); // Initialize with zeros

    AStar a_star(map.info.resolution, 8.0f);
    a_star.setMap(map.data, map.info.width, map.info.height);
    a_star.setStartPoint(0.0f, 0.0f, 0.0f);
    a_star.setGoal(9.0f, 9.0f, 0.0f);

    nav_msgs::msg::Path path;

    std::vector<Point> path_points;
    if (a_star.getPath(path_points))
    {
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
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No path found");
    }

    rclcpp::spin(std::make_shared<MinimalPublisher>(map, path));
    rclcpp::shutdown();
    return 0;
}