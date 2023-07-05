#include <iostream>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace odom_to_tf
{
using nav_msgs::msg::Odometry;

class Odom2TF : public rclcpp::Node        
{
public:
  Odom2TF(rclcpp::NodeOptions options): Node("odom_to_tf", options), br(this)
  {
    const auto odom_topic{declare_parameter<std::string>("/swift/odom", "odom")};

    if(odom_topic.empty())
    {
      RCLCPP_ERROR(get_logger(), "Passed odom_topic is empty, cannot subscribe");
      return;
    }

    odom_sub = create_subscription<Odometry>(odom_topic, 1, [&](Odometry::SharedPtr msg)
    {republish(msg->header, msg->child_frame_id, msg->pose.pose);});
    RCLCPP_INFO(this->get_logger(), "world to odom frame initialized");
  }

private:  
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::msg::TransformStamped tf;

  void republish(const std_msgs::msg::Header &header,
                 const std::string &child_frame,
                 const geometry_msgs::msg::Pose &pose)
  {

    tf.header = header;
    tf.child_frame_id = child_frame;
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = pose.orientation;
    br.sendTransform(tf);
  }
};
}

// boilerplate main
int main(int argc, char** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<odom_to_tf::Odom2TF>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
