#include "rclcpp/rclcpp.hpp"
#include <uuv_control_msgs/srv/go_to.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "gazebo_msgs/msg/model_states.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
using namespace BT;
using std::placeholders::_1;
double yaw_reading = 0;
double pitch_reading = 0;
double roll_reading = 0;
double x_reading = 0;
double y_reading = 0;
double z_reading = 0;
double check_yaw;
double check_x;
double check_y;

class Robot_state : public rclcpp::Node
{
public:
  Robot_state() : Node("subscriber")
  {

    Oriantation_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(topic_, 100,
                                                                                       std::bind(&Robot_state::callbackCheckRotation, this, _1));
    RCLCPP_INFO(this->get_logger(), "Robot_state  initialized");
  }

private:
  void callbackCheckRotation(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {

    tf2::Quaternion q(
        msg->pose[3].orientation.x,
        msg->pose[3].orientation.y,
        msg->pose[3].orientation.z,
        msg->pose[3].orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    roll_reading = roll;
    pitch_reading = pitch;
    yaw_reading = yaw;

    x_reading = msg->pose[3].position.x;
    y_reading = msg->pose[3].position.y;
    z_reading = msg->pose[3].position.z;

    // std::cout << "yaw : " << yaw_reading << std::endl;
    // std::cout << "x   : " << x_reading << std::endl;
  }
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr Oriantation_subscriber_;
  
  std::string topic_ = "/gazebo/model_states";
};

class Check_Rotation : public StatefulActionNode
{
public:
  Check_Rotation(const std::string &name, const NodeConfiguration &config): StatefulActionNode(name, config){}

  static BT::PortsList providedPorts() { return {}; }

  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;
private:
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("check_rotation_node");
  rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Rotate_Client_;
};

class Check_point : public StatefulActionNode
{
public:
  Check_point(const std::string &name, const NodeConfiguration &config) : StatefulActionNode(name, config){ }
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus onStart() override ;
  NodeStatus onRunning() override ;
  void onHalted() override ;

private : 
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("check_point_node");
  rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Move_Forward_Client_;
};

class Check_left : public StatefulActionNode
{
public:
  Check_left(const std::string &name, const NodeConfiguration &config) : StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("check_point_node");
  rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Move_Forward_Client_;
};

class IF_Subscriber_On : public StatefulActionNode{
  public:
    IF_Subscriber_On(const std::string &name, const NodeConfiguration &config) : StatefulActionNode(name, config){} 
    static BT::PortsList providedPorts() { return {}; }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

   

};

NodeStatus Check_point::onStart(){

        std::cout<< "onstartpoint" << std::endl;
        this->Move_Forward_Client_ = this->node_->create_client<uuv_control_msgs::srv::GoTo>("/rexrov/go_to");
        while (!this->Move_Forward_Client_->wait_for_service(std::chrono::seconds(1)))
        {
          std::cout << "waiting for forward service to be up...." << std::endl;
        }
        auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
        std::cout << "target x:" << x_reading + 6 << "target y:" << y_reading     << "target z:" << z_reading  <<"yaw     :" << yaw_reading << std::endl;
        request->waypoint.point.set__x(x_reading+6);
        request->waypoint.point.set__y(y_reading);
        request->waypoint.point.set__z(z_reading);
        request->waypoint.set__max_forward_speed(0.2);
        request->waypoint.set__heading_offset(0);
        request->waypoint.set__use_fixed_heading(true);
        this->Move_Forward_Client_->async_send_request(request);
        check_x=x_reading+6;
        return BT::NodeStatus::RUNNING;

}

NodeStatus Check_point::onRunning()
    {
      // std::cout << "Check_point: " << this->name() << std::endl;
      if (x_reading > check_x-0.2)
      {
        // std::cout << "passed 6 x " << x_reading << std::endl;
        return BT::NodeStatus::SUCCESS;
      }
      else
      {
        // std::cout << "less than 6 x " << x_reading << std::endl;
        return BT::NodeStatus::RUNNING;
      }
    }

void Check_point::onHalted(){
      std::cout<< "haluted"<<std::endl;
    }

NodeStatus Check_left::onStart()
{

  std::cout << "onstartpoint" << std::endl;
  this->Move_Forward_Client_ = this->node_->create_client<uuv_control_msgs::srv::GoTo>("/rexrov/go_to");
  while (!this->Move_Forward_Client_->wait_for_service(std::chrono::seconds(1)))
  {
    std::cout << "waiting for forward service to be up...." << std::endl;
  }
  auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
  std::cout << "target x:" << x_reading << "target y:" << y_reading + 6 << "target z:" << z_reading << "yaw     :" << yaw_reading << std::endl;
  request->waypoint.point.set__x(x_reading);
  request->waypoint.point.set__y(y_reading+6);
  request->waypoint.point.set__z(z_reading);
  request->waypoint.set__max_forward_speed(0.2);
  request->waypoint.set__heading_offset(0);
  request->waypoint.set__use_fixed_heading(true);
  this->Move_Forward_Client_->async_send_request(request);
  check_y = y_reading + 6;
  return BT::NodeStatus::RUNNING;
}

NodeStatus Check_left::onRunning()
{
  // std::cout << "Check_point: " << this->name() << std::endl;
  if (y_reading > check_y-0.2)
  {
    // std::cout << "passed 6 y " << x_reading << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    // std::cout << "less than 6 y " << x_reading << std::endl;
    return BT::NodeStatus::RUNNING;
  }
}

void Check_left::onHalted()
{
  std::cout << "haluted" << std::endl;
}

NodeStatus Check_Rotation::onStart(){
  std::cout<<"onstartrotate"<<std::endl;
  this->Rotate_Client_ = this->node_->create_client<uuv_control_msgs::srv::GoTo>("/rexrov/go_to");
  while (!this->Rotate_Client_->wait_for_service(std::chrono::seconds(1)))
  {
        std::cout << "waiting for rotate service to be up...." << std::endl;
  }
  auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
  std::cout << "target x:" << x_reading << "target y:" << y_reading << "target z:" << z_reading << "yaw     :" << yaw_reading +0.4 << std::endl;
  request->waypoint.point.set__x(x_reading);
  request->waypoint.point.set__y(y_reading);
  request->waypoint.point.set__z(z_reading);
  request->waypoint.set__max_forward_speed(0.2);
  request->waypoint.set__heading_offset(yaw_reading+0.4);
  request->waypoint.set__use_fixed_heading(false);
  this->Rotate_Client_->async_send_request(request);
  check_yaw = yaw_reading + 0.4;
  return BT::NodeStatus::RUNNING;
}

NodeStatus Check_Rotation::onRunning()
{
    // std::cout << "Check_Rotation: " << this->name() << std::endl;
    if (yaw_reading > check_yaw + 0.05)
    {
        // std::cout << "passed 0.4 yaw " << yaw_reading << std::endl;
        return BT::NodeStatus::SUCCESS;
        }
        else
        {
      // std::cout << "less than 0.4 yaw: " << yaw_reading << std::endl;

      return BT::NodeStatus::RUNNING;
        }
    }

void Check_Rotation::onHalted(){
  std::cout << "haluted" << std::endl;
  }

NodeStatus IF_Subscriber_On::onStart()
{
  std::cout << "on startcheck subscriber" << std::endl;
  return BT::NodeStatus::RUNNING;
}

NodeStatus IF_Subscriber_On::onRunning()
{
  if (x_reading != 0 && y_reading != 0 && z_reading != 0)
  {
      std::cout << "subscriber on" << std::endl;
      return BT::NodeStatus::SUCCESS;
  }
  else
  {
      return BT::NodeStatus::RUNNING;
  }
}

void IF_Subscriber_On::onHalted()
{
  std::cout << "haluted" << std::endl;
}

static const char *xml_text_medium = R"(
 <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
        <Fallback name="main_fallback">
            <Sequence name="move_after_rotation">
                <Action ID="IF_Subscriber_On"/>
                <Action ID="Check_left"/>
                <Action ID="Check_point"/>
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>
 )";

//  <Action ID="Check_Rotation"/>
        int main(int argc, char **argv)
        {
      rclcpp::init(argc, argv);
      // auto nh = std::make_shared<rclcpp::Node>("sleep_client");
      BehaviorTreeFactory factory;

      factory.registerNodeType<IF_Subscriber_On>("IF_Subscriber_On");

      factory.registerNodeType<Check_left>("Check_left");
      // factory.registerNodeType<Check_Rotation>("Check_Rotation");
      factory.registerNodeType<Check_point>("Check_point");

      std::cout << "\n------------ BUILDING A NEW TREE ------------" << std::endl;

      auto tree = factory.createTreeFromText(xml_text_medium);
      std::cout << "\n------------ set ZMQ Publisher   ------------\n"
                << std::endl;

      unsigned max_msg_per_second = 25;
      unsigned publisher_port = 1666;
      unsigned server_port = 1667;
      PublisherZMQ publisher_zmq(tree, max_msg_per_second, publisher_port, server_port);
      NodeStatus status = NodeStatus::RUNNING;
      BT::NodeConfiguration con = {};

      auto node = std::make_shared<Robot_state>();

      while (status != BT::NodeStatus::SUCCESS)
      {

        rclcpp::spin_some(node);

        // tree.haltTree();

        status = tree.tickRoot();

        // std::this_thread::sleep_for(std::chrono::milliseconds(50));

        // tree.tickRoot();
        //  tree.sleep(std::chrono::milliseconds(50));
      }

      return 0;
        }
