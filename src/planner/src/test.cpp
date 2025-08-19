#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
    MinimalPublisher(nav_msgs::msg::OccupancyGrid map)
        : Node("map_publisher"), map_(map)
    {
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("topic", 10);
        timer_ = this->create_wall_timer(
            500ms, std::bind(&MinimalPublisher::timer_callback, this));
    }

private:
    void timer_callback()
    {
        map_publisher_->publish(map_);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
    nav_msgs::msg::OccupancyGrid map_;
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

    rclcpp::spin(std::make_shared<MinimalPublisher>(map));
    rclcpp::shutdown();
    return 0;
}