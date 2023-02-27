#include "rclcpp/rclcpp.hpp"
#include <uuv_control_msgs/srv/go_to.hpp>

class SetWayPointNode : public rclcpp::Node
{
   public:
     SetWayPointNode() : Node("setwaypoint")
     {
       Client_ = this->create_client<uuv_control_msgs::srv::GoTo>("/swift/go_to");
       while (!Client_->wait_for_service(std::chrono::seconds(1)))
       {
         RCLCPP_WARN(this->get_logger(), "waiting for service to be up....");
       }
       RCLCPP_INFO(this->get_logger(), "client created");

       send_waypoint(4, 2, -0.75, 0.2, 0);
     }
   void send_waypoint(float x, float y, float z, float max_speed, float heading)
       {
       auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
       request->waypoint.point.set__x(x);
       request->waypoint.point.set__y(y);
       request->waypoint.point.set__z(z);
       request->waypoint.set__max_forward_speed(max_speed);
       request->waypoint.set__heading_offset(heading);
       request->waypoint.set__use_fixed_heading(true);
       Client_->async_send_request(request);
       }
       rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Client_;
};



  int main(int argc, char **argv)
  {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SetWayPointNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
  }
  
