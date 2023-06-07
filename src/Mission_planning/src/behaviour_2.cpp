#include <iostream>
#include <chrono>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <uuv_control_msgs/srv/go_to.hpp>
#include "gazebo_msgs/msg/model_states.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace BT;
using std::placeholders::_1;
bool idle_forward = true;
bool idle_rotate = true;

class Check_Rotation : public AsyncActionNode
{
public:
  double y_reading;
  Check_Rotation(const std::string &name, const NodeConfiguration &config)
      : AsyncActionNode(name, config) {
    std::thread thread_spin(Check_Rotation::spin, node_);
    thread_spin.detach();
    std::cout << "after spin rotate" << std::endl;
      }

  static PortsList providedPorts()
  {
    return {};
  }
  static void spin(rclcpp::Node::SharedPtr node)
  {
    std::cout << "in spin rotate" << std::endl;
    rclcpp::spin(node);
  }

  void subscribe_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    
    y_reading = msg->pose[3].position.y;
  }

  NodeStatus tick() override
  {
    std::cout << "rotate state:" << status()<< std::endl;
 
    if (status() == NodeStatus::IDLE || idle_rotate==true)
    {
      idle_rotate=false;
              std::cout
          << "in rotate idle" << std::endl;
      subscriber_ = node_->create_subscription<gazebo_msgs::msg::ModelStates>(
          topic_, 100, std::bind(&Check_Rotation::subscribe_callback, this, _1));
      std::cout << "rotate subscriber init" << std::endl;
      setOutput<bool>("success", false);
      setOutput<bool>("finished", false);
      Rotate_Client_ = node_->create_client<uuv_control_msgs::srv::GoTo>(service_);
      while (!Rotate_Client_->wait_for_service(std::chrono::seconds(1)))
      {
        std::cout << "waiting for service to be up...." << std::endl;
       
      }
      std::cout << "client created" << std::endl;
      auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
             request->waypoint.point.set__x(0);
             request->waypoint.point.set__y(6);
             request->waypoint.point.set__z(-20);
             request->waypoint.set__max_forward_speed(0.2);
             request->waypoint.set__heading_offset(0);
             request->waypoint.set__use_fixed_heading(true);
             Rotate_Client_->async_send_request(request);
          
             setStatus(NodeStatus::RUNNING);
    }
    else if (status() == NodeStatus::RUNNING)
    {

      if (y_reading > 5)
      {
        subscriber_.reset();
        setStatus(NodeStatus::SUCCESS);
        std::cout << "in rotate success" << std::endl;
      }

    }
    
    return status();
  }


private:
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("check_rotation_node");
  rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Rotate_Client_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscriber_;
  std::string topic_ = "/gazebo/model_states";
  std::string service_ = "/rexrov/go_to";
};

class Move_Forward : public AsyncActionNode
{
public:
  double x_reading;
  Move_Forward(const std::string &name, const NodeConfiguration &config)
      : AsyncActionNode(name, config)
  {
    std::thread thread_spin(Check_Rotation::spin, node_);
    thread_spin.detach();
    std::cout << "after spin forward" << std::endl;
  }
  static void spin(rclcpp::Node::SharedPtr node)
  {
    std::cout << "in spin forward" << std::endl;
    rclcpp::spin_some(node);
  }
  static PortsList providedPorts()
  {
    return {};
  }

  void subscribe_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    std::cout << "position update " << std::endl;
    x_reading = msg->pose[3].position.x;

  }

  NodeStatus tick() override
  {
    std::cout << "in tick forward" << std::endl;
    std::cout << "forward state:" << status() << std::endl;
    if (status() == NodeStatus::IDLE || idle_forward== true)
    {
      idle_forward=false;
      std::cout << "in forward idle" << std::endl;
      subscriber_ = node_->create_subscription<gazebo_msgs::msg::ModelStates>(
          topic_, 100, std::bind(&Move_Forward::subscribe_callback, this, _1));
      std::cout << "forward subscriber init" << std::endl;
      setOutput<bool>("success", false);
      setOutput<bool>("finished", false);
      Forward_Client_ = node_->create_client<uuv_control_msgs::srv::GoTo>(service_);
      while (!Forward_Client_->wait_for_service(std::chrono::seconds(1)))
      {
        std::cout << "waiting for service to be up...." << std::endl;
      }
      std::cout << "client created" << std::endl;
      auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
      request->waypoint.point.set__x(6);
      request->waypoint.point.set__y(6);
      request->waypoint.point.set__z(-20);
      request->waypoint.set__max_forward_speed(0.2);
      request->waypoint.set__heading_offset(0);
      request->waypoint.set__use_fixed_heading(true);
      Forward_Client_->async_send_request(request);
 
      setStatus(NodeStatus::RUNNING);
    }
    else if (status() == NodeStatus::RUNNING)
    {
        
      if (x_reading > 5)
      {
        subscriber_.reset();
        setStatus(NodeStatus::SUCCESS);
        std::cout << "in forward success" << std::endl;
      }
    }
    return status();
  }


private:
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("move_forward_node");
  rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr  Forward_Client_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscriber_;
  std::string topic_ = "/gazebo/model_states";
  std::string service_ = "/rexrov/go_to";
};

static const char *xml_text_medium = R"(

 <root main_tree_to_execute="BehaviorTree">

    <BehaviorTree ID="BehaviorTree">
        <Fallback name="root">
            <Sequence name="move_after_rotation">
                <Action ID="Check_Rotation"/>
                <Action ID="Move_Forward"/>
            </Sequence>
        </Fallback>
    </BehaviorTree>
 
</root>
 )";


int main(int argc, char **argv)
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
        std::cout << "in main" << std::endl;
  // Create BehaviorTree factory
  BehaviorTreeFactory factory;

  // Register nodes
  factory.registerNodeType<Check_Rotation>("Check_Rotation");
  factory.registerNodeType<Move_Forward>("Move_Forward");

  auto node = rclcpp::Node::make_shared("behaviour_node");
  // Create tree

  auto tree = factory.createTreeFromText(xml_text_medium);

  // Create ZMQ publisher for logging

  unsigned max_msg_per_second = 25;
  unsigned publisher_port = 1666;
  unsigned server_port = 1667;
  PublisherZMQ publisher_zmq(tree, max_msg_per_second, publisher_port, server_port);
  // PublisherZMQ publisher_zmq(tree);



  // Run tree
  NodeStatus status = NodeStatus::RUNNING;
  while (status == NodeStatus::RUNNING)
  {
           
    status = tree.rootNode()->executeTick();

  }

  // Shutdown ROS2
  rclcpp::shutdown();

  return 0;
}
