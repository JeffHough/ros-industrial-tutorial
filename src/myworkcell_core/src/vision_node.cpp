/**
**  Simple ROS Node
**/
#include <rclcpp/rclcpp.hpp>
#include <fake_ar_publisher/msg/ar_marker.hpp>
#include <myworkcell_core/srv/localize_part.hpp>

// includes to be able to work with geometry msgs and the transform listener:
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// the message definitions themselves:
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Localizer : public rclcpp::Node
{
public:
  Localizer() : Node("vision_node"), last_msg_{nullptr}, buffer_(get_clock()), listener_(buffer_)
  {
    ar_sub_ = this->create_subscription<fake_ar_publisher::msg::ARMarker>(
        "ar_pose_marker",
        rclcpp::QoS(1),
        std::bind(&Localizer::visionCallback, this, std::placeholders::_1));

    server_ = this->create_service<myworkcell_core::srv::LocalizePart>(
        "localize_part",
        std::bind(&Localizer::localizePart, this, std::placeholders::_1, std::placeholders::_2));
  }

  void visionCallback(fake_ar_publisher::msg::ARMarker::SharedPtr msg)
  {
    last_msg_ = msg;
    RCLCPP_INFO(get_logger(), "Received pose: x=%f, y=%f, z=%f",
                msg->pose.pose.position.x,
                msg->pose.pose.position.y,
                msg->pose.pose.position.z);
  }

  void localizePart(
      myworkcell_core::srv::LocalizePart::Request::SharedPtr req,
      myworkcell_core::srv::LocalizePart::Response::SharedPtr res)
  {
    // Read the last message:
    fake_ar_publisher::msg::ARMarker::SharedPtr p = last_msg_;

    if (!p)
    {
      RCLCPP_ERROR(get_logger(), "no data");
      res->success = false;
      return;
    }

    geometry_msgs::msg::PoseStamped target_pose_from_cam;
    target_pose_from_cam.header = p->header;
    target_pose_from_cam.pose = p->pose.pose;
    
    // here, we transform the target pose from the request into the frame they asked for.
    geometry_msgs::msg::PoseStamped target_pose_from_req = buffer_.transform(
        target_pose_from_cam, req->base_frame);
    
    res->success = true;
  }

  rclcpp::Subscription<fake_ar_publisher::msg::ARMarker>::SharedPtr ar_sub_;
  fake_ar_publisher::msg::ARMarker::SharedPtr last_msg_;
  rclcpp::Service<myworkcell_core::srv::LocalizePart>::SharedPtr server_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Localizer>();

  RCLCPP_INFO(node->get_logger(), "Vision node starting");

  // Don't exit the program.
  rclcpp::spin(node);
}
