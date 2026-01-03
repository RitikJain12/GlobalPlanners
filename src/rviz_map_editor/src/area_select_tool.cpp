#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include "rviz_rendering/viewport_projection_finder.hpp"
#include "rviz_common/interaction/selection_manager.hpp"

#include <OgreViewport.h>
#include <OgreCamera.h>
#include <OgreRay.h>
#include <OgrePlane.h>
#include <OgreVector3.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include <pluginlib/class_list_macros.hpp>

class AreaSelectTool : public rviz_common::Tool
{
public:
    AreaSelectTool()
    {
        projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    }

    void onInitialize() override
    {
        node_ = context_->getRosNodeAbstraction().lock()->get_raw_node();
        pub_ = node_->create_publisher<geometry_msgs::msg::PolygonStamped>(
            "/add_polygon", 10);
    }

    void activate() override
    {
        context_->getSelectionManager()->setTextureSize(512);
        selection_ = false;
        activated_ = true;
    }
    void deactivate() override
    {
        activated_ = false;
        context_->getSelectionManager()->removeHighlight();
    }

    int processMouseEvent(rviz_common::ViewportMouseEvent &event) override
    {
        if (!activated_)
        {
            return 0;
        }

        auto sel_manager = context_->getSelectionManager();

        if (event.leftDown())
        {
            selection_ = true;
            sel_start_x_ = event.x;
            sel_start_y_ = event.y;
            start_ = getPoint(event);
        }

        if (event.leftUp())
        {
            selection_ = false;
            auto end = getPoint(event);
            publishRectangle(start_, end);
        }

        if (selection_)
        {
            sel_manager->highlight(
                event.panel->getRenderWindow(),
                sel_start_x_,
                sel_start_y_,
                event.x,
                event.y);
        }
        else
        {
            sel_manager->removeHighlight();
        }
        return Render;
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_;
    std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
    Ogre::Vector3 start_;
    bool activated_ = false;
    bool selection_ = false;
    int sel_start_x_;
    int sel_start_y_;

    Ogre::Vector3 getPoint(rviz_common::ViewportMouseEvent &event)
    {

        auto point_projection_on_xy_plane = projection_finder_->getViewportPointProjectionOnXYPlane(
            event.panel->getRenderWindow(), event.x, event.y);

        return point_projection_on_xy_plane.second;
    }

    void publishRectangle(const Ogre::Vector3 &a, const Ogre::Vector3 &b)
    {
        geometry_msgs::msg::PolygonStamped poly;
        poly.header.frame_id = "map";

        poly.polygon.points.resize(4);

        poly.polygon.points[0].x = a.x;
        poly.polygon.points[0].y = a.y;

        poly.polygon.points[1].x = b.x;
        poly.polygon.points[1].y = a.y;

        poly.polygon.points[2].x = b.x;
        poly.polygon.points[2].y = b.y;

        poly.polygon.points[3].x = a.x;
        poly.polygon.points[3].y = b.y;

        pub_->publish(poly);
    }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(AreaSelectTool, rviz_common::Tool)