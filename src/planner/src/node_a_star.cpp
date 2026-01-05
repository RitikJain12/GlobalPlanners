#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "a_star.h"
#include "map.h"

using namespace std::chrono_literals;

class PlannerAStar : public rclcpp::Node
{
public:
    PlannerAStar() : Node("astar_planner")
    {
        map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
        path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        footprint_publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("/footprint", 10);

        this->declare_parameter("planner.type", "AStar");

        this->declare_parameter("planner.theta_resolution", 8.0);
        this->declare_parameter("planner.xy_tollerance", 0.1);
        this->declare_parameter("planner.theta_tollerance", 0.1);

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

        start_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1,
            std::bind(&PlannerAStar::startCallback, this, std::placeholders::_1));

        obstacle_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
            "/add_polygon", 1,
            std::bind(&PlannerAStar::obstacleCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            500ms, std::bind(&PlannerAStar::timer_callback, this));
    }

    ~PlannerAStar()
    {
        delete a_star_;
    }

private:
    Point getPoint(const geometry_msgs::msg::Pose pose_msg)
    {
        return Point(pose_msg.position.x, pose_msg.position.y,
                     2 * atan2(pose_msg.orientation.z, pose_msg.orientation.w));
    }

    void initialize_env()
    {
        float map_width;  // in meters
        float map_height; // in meters
        float resoluion;

        this->get_parameter("map.width", map_width);
        this->get_parameter("map.height", map_height);
        this->get_parameter("map.resolution", resoluion);

        nav_msgs::msg::OccupancyGrid map;
        map_.header.frame_id = "map";
        map_.info.resolution = resoluion;
        map_.info.width = static_cast<int>(ceil(map_width / map_.info.resolution));   // in cells
        map_.info.height = static_cast<int>(ceil(map_height / map_.info.resolution)); // in cells

        map_ptr_ = std::make_shared<Map>(map_width, map_height, map_.info.resolution);

        map_.data = map_ptr_->getMap();

        std::vector<Point> footprint = {Point(-1.0, 0.5), Point(-1.0, -0.5), Point(2.0, -0.5), Point(2.0, 0.5)};
        map_ptr_->setFootprint(footprint);
    }

    void set_footprint(Point start)
    {
        std::vector<std::pair<float, float>> fp = map_ptr_->getFootprint(start);

        footprint_.polygon.points.clear();
        for (const std::pair<float, float> &p : fp)
        {
            geometry_msgs::msg::Point32 fp_point;
            fp_point.x = p.first;
            fp_point.y = p.second;
            footprint_.polygon.points.push_back(fp_point);
        }
        footprint_.header.frame_id = "map";
    }

    void initialize_planner()
    {
        float theta_resolution;
        float xy_tollerance;
        float theta_tollerance;

        if (!this->get_parameter("planner.theta_resolution", theta_resolution))
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Defaulting theta resolution to 8.0");
            theta_resolution = 8.0;
        }

        if (!this->get_parameter("planner.xy_tollerance", xy_tollerance))
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Defaulting distance tollerance to 0.1");
            xy_tollerance = 0.1;
        }

        if (!this->get_parameter("planner.theta_tollerance", theta_tollerance))
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Defaulting theta tollerance to 0.1");
            theta_tollerance = 0.1;
        }

        std::string planner_type;
        this->get_parameter("planner.type", planner_type);

        if (planner_type == "HybridAStar")
        {
            bool use_dynamic;
            bool allow_reverse;
            float wheelbase;
            float max_velocity;
            float min_linear_acc;
            float steer_resolution;
            float max_steer;

            this->get_parameter("planner.use_dynamic", use_dynamic);
            this->get_parameter("planner.allow_reverse", allow_reverse);
            this->get_parameter("planner.wheelbase", wheelbase);
            this->get_parameter("planner.max_velocity", max_velocity);
            this->get_parameter("planner.min_linear_acc", min_linear_acc);
            this->get_parameter("planner.steer_resolution", steer_resolution);
            this->get_parameter("planner.max_steer", max_steer);
        }
        else
        {
            a_star_ = new AStar(map_ptr_, theta_resolution, xy_tollerance, theta_tollerance);
        }

        Point start = Point(1.0, 1.5, 1.57);
        a_star_->setStartPoint(start);

        set_footprint(start);
    }

    void try_plan()
    {
        std::vector<Point> path_points;
        path_ = nav_msgs::msg::Path();

        if (!a_star_->getPath(path_points))
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No path found");
            return;
        }

        path_.header.frame_id = "map";
        for (const auto &point : path_points)
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.orientation.z = sin(point.theta / 2.0);
            pose.pose.orientation.w = cos(point.theta / 2.0);
            // std::cout << "Path point: (" << point.x << ", " << point.y << ", " << point.theta << ")" << std::endl;
            path_.poses.push_back(pose);
        }
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped &msg)
    {
        a_star_->setGoal(getPoint(msg.pose));
        try_plan();
    }

    void startCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
    {
        Point start = Point(getPoint(msg.pose.pose));
        a_star_->setStartPoint(start);
        set_footprint(start);
    }

    void obstacleCallback(const geometry_msgs::msg::PolygonStamped &msg)
    {
        std::vector<Point> obstacle;
        for (geometry_msgs::msg::Point32 point : msg.polygon.points)
        {
            obstacle.push_back(Point(point.x, point.y));
        }
        map_ptr_->setObstacles(obstacle);
        map_.data = map_ptr_->getMap();
    }

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
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr start_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr obstacle_sub_;

    nav_msgs::msg::OccupancyGrid map_;
    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PolygonStamped footprint_;

    std::shared_ptr<Map> map_ptr_;
    AStar *a_star_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<PlannerAStar>());
    rclcpp::shutdown();
    return 0;
}