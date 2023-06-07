#include "rclcpp/rclcpp.hpp"
#include <uuv_control_msgs/srv/go_to.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "gazebo_msgs/msg/model_states.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
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
double check_z;
double gate_area;
bool gate_detected = false;
double gate_x;
double gate_y;
double initial_x;
std::string robot_name = "rexrov";
class Robot_state : public rclcpp::Node
{
public:
  Robot_state() : Node("subscriber")
  {

    Oriantation_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(topic_, 100,
                                                                                       std::bind(&Robot_state::callbackCheckRotation, this, _1));
    RCLCPP_INFO(this->get_logger(), "Robot_state  initialized");

    Gate_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(gate_topic_, 100,
                                                                          std::bind(&Robot_state::callDetectGate, this, _1));
    RCLCPP_INFO(this->get_logger(), "gate_state  initialized");
  }

private:
  void callbackCheckRotation(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {

    tf2::Quaternion q(
        msg->pose[2].orientation.x,
        msg->pose[2].orientation.y,
        msg->pose[2].orientation.z,
        msg->pose[2].orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    roll_reading = roll;
    pitch_reading = pitch;
    yaw_reading = yaw;

    x_reading = msg->pose[2].position.x;
    y_reading = msg->pose[2].position.y;
    z_reading = msg->pose[2].position.z;

    // std::cout << "yaw : " << yaw_reading << std::endl;
    // std::cout << "x   : " << x_reading << std::endl;
  }
  void callDetectGate(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->pose.pose.position.z>0)
      gate_detected=true;
    else
    gate_detected=false;

    gate_x = msg->twist.twist.linear.x - msg->pose.pose.position.x;
    gate_x = msg->twist.twist.linear.y - msg->pose.pose.position.y;
    if ((gate_x * gate_y)>400){
    gate_area = gate_x * gate_y;
    }
    gate_area= gate_x*gate_y;
    gate_x = msg->pose.pose.position.x + gate_x/2;
    gate_y = msg->pose.pose.position.y + gate_y/2;

    std::cout << "area : " << gate_area << std::endl;
    std::cout << "gate_x   : " << gate_x <<  " gate_y   : " << gate_y << std::endl;
  }
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr Oriantation_subscriber_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Gate_subscriber_;

  std::string topic_ = "/gazebo/model_states";
  std::string gate_topic_ = "/Gate_position";
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

class Move_Forward : public StatefulActionNode
{
public:
  Move_Forward(const std::string &name, const NodeConfiguration &config) : StatefulActionNode(name, config){ }
  static BT::PortsList providedPorts() { return {InputPort<std::string>("Value")}; }
  NodeStatus onStart() override ;
  NodeStatus onRunning() override ;
  void onHalted() override ;

private : 
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("Move_Forward_node");
  rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Move_Forward_Client_;
};

class Move_Left : public StatefulActionNode
{
public:
  Move_Left(const std::string &name, const NodeConfiguration &config) : StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {InputPort<std::string>("Value")}; }
  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("Move_Forward_node");
  rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Move_Forward_Client_;
};

class Move : public StatefulActionNode
{
public:
  Move(const std::string &name, const NodeConfiguration &config) : StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {InputPort<std::string>("Value"),  InputPort<std::string>("axis")}; }
  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("Move_node");
  rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Move_Client_;
};

class Move_Down : public StatefulActionNode
{
public:
  Move_Down(const std::string &name, const NodeConfiguration &config) : StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  NodeStatus onStart() override;
  NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_ = rclcpp::Node::make_shared("Move_Down_node");
  rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Move_Down_Client_;
};

class IF_Subscriber_On : public StatefulActionNode{
  public:
    IF_Subscriber_On(const std::string &name, const NodeConfiguration &config) : StatefulActionNode(name, config){} 
    static BT::PortsList providedPorts() { return {}; }
    NodeStatus onStart() override;
    NodeStatus onRunning() override;
    void onHalted() override;

   

};

NodeStatus IF_Gate_Visible()
{
    if (gate_detected)
    return BT::NodeStatus::SUCCESS;
    else
    return BT::NodeStatus::FAILURE;
}

NodeStatus IF_Gate_Centered()
{
    if (gate_x>200 && gate_x<300  )
    return BT::NodeStatus::SUCCESS;
    else
    return BT::NodeStatus::FAILURE;
}

NodeStatus IF_Gate_Right()
{
    if (gate_x > 300)
    return BT::NodeStatus::SUCCESS;
    else
    return BT::NodeStatus::FAILURE;
}
NodeStatus IF_Gate_Left()
{
    if (gate_x < 200)
    return BT::NodeStatus::SUCCESS;
    else
    return BT::NodeStatus::FAILURE;
}

NodeStatus IF_Gate_Passed()
{
    if (gate_area < 2000 && gate_detected)
    return BT::NodeStatus::SUCCESS;
    else
    return BT::NodeStatus::FAILURE;
}

NodeStatus Move_Forward::onStart(){

        std::cout<< "onstartpoint" << std::endl;
        this->Move_Forward_Client_ = this->node_->create_client<uuv_control_msgs::srv::GoTo>("/"+robot_name+"/go_to");
        while (!this->Move_Forward_Client_->wait_for_service(std::chrono::seconds(1)))
        {
          std::cout << "waiting for forward service to be up...." << std::endl;
        }
        auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
        auto value = getInput<std::string>("Value").value();
        check_x = x_reading + std::stod(value);

        std::cout << "target x:" << check_x << "target y:" << y_reading << "target z:" << z_reading << "yaw     :" << yaw_reading << std::endl;
        request->waypoint.point.set__x(check_x);
        request->waypoint.point.set__y(y_reading);
        request->waypoint.point.set__z(z_reading);
        request->waypoint.set__max_forward_speed(0.2);
        request->waypoint.set__heading_offset(0);
        request->waypoint.set__use_fixed_heading(false);
        this->Move_Forward_Client_->async_send_request(request);
        
        return BT::NodeStatus::RUNNING;

}

NodeStatus Move_Forward::onRunning()
    {
      // std::cout << "Move_Forward: " << this->name() << std::endl;
      if (x_reading > check_x)
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

void Move_Forward::onHalted(){
      std::cout<< "haluted"<<std::endl;
    }

NodeStatus Move_Down::onStart(){

        std::cout<< "onstartpoint" << std::endl;
        this->Move_Down_Client_ = this->node_->create_client<uuv_control_msgs::srv::GoTo>("/"+robot_name+"/go_to");
        while (!this->Move_Down_Client_->wait_for_service(std::chrono::seconds(1)))
        {
          std::cout << "waiting for forward service to be up...." << std::endl;
        }
        auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
        check_z = z_reading + -0.5;
        std::cout << "target x:" << x_reading << "target y:" << y_reading << "target z:" << check_z << "  yaw:" << yaw_reading << std::endl;
        request->waypoint.point.set__x(x_reading+0.1);
        request->waypoint.point.set__y(y_reading);
        request->waypoint.point.set__z(check_z);
        request->waypoint.set__max_forward_speed(0.2);
        request->waypoint.set__heading_offset(0);
        request->waypoint.set__use_fixed_heading(true);
        this->Move_Down_Client_->async_send_request(request);
        
        return BT::NodeStatus::RUNNING;

}

NodeStatus Move_Down::onRunning()
    {
      // std::cout << "Move_Down: " << this->name() << std::endl;
        if (z_reading <= check_z )
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

void Move_Down::onHalted(){
      std::cout<< "haluted"<<std::endl;
    }

NodeStatus Move_Left::onStart()
{

  std::cout << "onstartpoint" << std::endl;
  this->Move_Forward_Client_ = this->node_->create_client<uuv_control_msgs::srv::GoTo>("/"+robot_name+"/go_to");
  while (!this->Move_Forward_Client_->wait_for_service(std::chrono::seconds(1)))
  {
    std::cout << "waiting for forward service to be up...." << std::endl;
  }
  auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
  auto value = getInput<std::string>("Value").value();
  check_y = y_reading + std::stod(value);
  std::cout << "target x:" << x_reading << "target y:" << check_y << "target z:" << z_reading << "yaw     :" << yaw_reading << std::endl;
  request->waypoint.point.set__x(x_reading);
  request->waypoint.point.set__y(check_y);
  request->waypoint.point.set__z(z_reading);
  request->waypoint.set__max_forward_speed(0.2);
  request->waypoint.set__heading_offset(-1.5707);
  request->waypoint.set__use_fixed_heading(false);
  
  this->Move_Forward_Client_->async_send_request(request);
  
  return BT::NodeStatus::RUNNING;
}

NodeStatus Move_Left::onRunning()
{
  // std::cout << "Move_Forward: " << this->name() << std::endl;
  if (y_reading > check_y)
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

void Move_Left::onHalted()
{
  std::cout << "haluted" << std::endl;
}

NodeStatus Move::onStart()
{

  std::cout << "onstartpoint" << std::endl;
  this->Move_Client_ = this->node_->create_client<uuv_control_msgs::srv::GoTo>("/"+robot_name+"/go_to");
  while (!this->Move_Client_->wait_for_service(std::chrono::seconds(1)))
  {
    std::cout << "waiting for forward service to be up...." << std::endl;
  }
  auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
  auto axis = std::stoi(getInput<std::string>("axis").value());
  auto value =std::stod(getInput<std::string>("Value").value());
  check_y = y_reading;
  check_x = x_reading;
  check_z = z_reading; 
  double offset=0.0001;
  switch (axis)
  {
  case 0:
    check_x = x_reading + value;
    break;
    case 1:
    if (value < 0) offset = 1.5707;
    else offset = -1.5707;
    check_y = y_reading + value;
  
    break;
    case 2:
    check_x = x_reading + 0.01;
    check_z = z_reading + value;
    break;
    default:
    std::cout<<"no axis"<<std::endl;
    }
 

    std::cout << "target x:" << check_x << "target y:" << check_y << "target z:" << check_z << "yaw     :" << yaw_reading << std::endl;
    request->waypoint.point.set__x(check_x);
    request->waypoint.point.set__y(check_y);
    request->waypoint.point.set__z(check_z);
    request->waypoint.set__max_forward_speed(0.2);
    request->waypoint.set__heading_offset(offset);
    request->waypoint.set__use_fixed_heading(true);

    this->Move_Client_->async_send_request(request);

    return BT::NodeStatus::RUNNING;
}

NodeStatus Move::onRunning()
{
  // std::cout << "Move_Forward: " << this->name() << std::endl;
    double x_tolerance=0.05 ;
    double y_tolerance=0.05;
    double z_tolerance=0.1;
    bool y_axis_reached = y_reading <= check_y + y_tolerance && y_reading >= check_y - y_tolerance;
    bool x_axis_reached = x_reading <= check_x + x_tolerance && x_reading >= check_x - x_tolerance;
    bool z_axis_reached = z_reading <= check_z + z_tolerance && z_reading >= check_z - z_tolerance;
    // std::cout << "reached x:" << x_axis_reached << "reached y:" << y_axis_reached << "reached z:" << z_axis_reached << std::endl;
    if (y_axis_reached && x_axis_reached && z_axis_reached)
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

void Move::onHalted()
{
  std::cout << "haluted" << std::endl;
}

NodeStatus Check_Rotation::onStart(){
  std::cout<<"onstartrotate"<<std::endl;
  this->Rotate_Client_ = this->node_->create_client<uuv_control_msgs::srv::GoTo>("/"+robot_name+"/go_to");
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
      initial_x=x_reading;
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
            <Sequence name="Follow_gate">
                <Action ID="IF_Subscriber_On"/>
                <RetryUntilSuccesful num_attempts="1000" name="search">
                    <ReactiveFallback name="search">
                        <Condition ID="IF_Gate_Visible"/>
                        <Inverter>
                            <Action ID="Move" axis="1" name="move_left" Value="2"/>
                        </Inverter>
                    </ReactiveFallback>
                </RetryUntilSuccesful>
                <RetryUntilSuccesful num_attempts="1000" name="centered">
                    <ReactiveFallback name="centered">
                        <Condition ID="IF_Gate_Centered"/>
                        <ReactiveSequence name="move_left">
                            <Condition ID="IF_Gate_Left"/>
                            <Inverter>
                                <Action ID="Move" axis="1" Value="0.5"/>
                            </Inverter>
                        </ReactiveSequence>
                        <ReactiveSequence name="move_right">
                            <Condition ID="IF_Gate_Right"/>
                            <Inverter>
                                <Action ID="Move" axis="1" Value="-0.5"/>
                            </Inverter>
                        </ReactiveSequence>
                    </ReactiveFallback>
                </RetryUntilSuccesful>
                <RetryUntilSuccesful num_attempts="1000" name="pass_gate">
                    <ReactiveFallback name="pass_gate">
                        <Condition ID="IF_Gate_Passed"/>
                        <ReactiveSequence name="move_forward">
                            <Condition ID="IF_Gate_Centered"/>
                            <Inverter>
                                <Action ID="Move" axis="0" Value="0.7"/>
                            </Inverter>
                        </ReactiveSequence>
                        <ReactiveSequence name="move_left">
                            <Condition ID="IF_Gate_Left"/>
                            <Inverter>
                                <Action ID="Move" axis="1" Value="0.5"/>
                            </Inverter>
                        </ReactiveSequence>
                        <ReactiveSequence name="move_right">
                            <Condition ID="IF_Gate_Right"/>
                            <Inverter>
                                <Action ID="Move" axis="1" Value="-0.5"/>
                            </Inverter>
                        </ReactiveSequence>
                    </ReactiveFallback>
                </RetryUntilSuccesful>
                <Action ID="Move" axis="0" Value="10"/>
            </Sequence>
        </Fallback>
    </BehaviorTree>
</root>
 )";

// static const char *xml_text_medium = R"(
//  <root main_tree_to_execute="BehaviorTree">
//     <BehaviorTree ID="BehaviorTree">
//         <Fallback name="main_fallback">
//             <Sequence name="Follow_gate">
//                 <Action ID="IF_Subscriber_On"/>

//                 <Action ID="Move" Value="1"  axis="0"/>
//                 <Action ID="Move" Value="1" axis="1"/>
//                 <Action ID="Move" Value="-1"  axis="1"/>
//                 <Action ID="Move" Value="-3"  axis="2"/>
//             </Sequence>
//         </Fallback>
//     </BehaviorTree>
// </root>
//  )";

//  <Action ID="Check_Rotation"/>
        int main(int argc, char **argv)
        {
      rclcpp::init(argc, argv);
      // auto nh = std::make_shared<rclcpp::Node>("sleep_client");
      BehaviorTreeFactory factory;

      factory.registerSimpleCondition("IF_Gate_Visible", std::bind(IF_Gate_Visible));
      factory.registerSimpleCondition("IF_Gate_Centered", std::bind(IF_Gate_Centered));
      factory.registerSimpleCondition("IF_Gate_Right", std::bind(IF_Gate_Right));
      factory.registerSimpleCondition("IF_Gate_Left", std::bind(IF_Gate_Left));
      factory.registerSimpleCondition("IF_Gate_Passed", std::bind(IF_Gate_Passed));

      factory.registerNodeType<IF_Subscriber_On>("IF_Subscriber_On");

      factory.registerNodeType<Move_Left>("Move_Left");
      // factory.registerNodeType<Check_Rotation>("Check_Rotation");
      factory.registerNodeType<Move_Forward>("Move_Forward");

      factory.registerNodeType<Move_Down>("Move_Down");

      factory.registerNodeType<Move>("Move");

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
      FileLogger logger_file(tree,"log_1.fbl");
      while (status != BT::NodeStatus::SUCCESS)
      {
      // std::cout << "status:" << status << std::endl;
      rclcpp::spin_some(node);

      // tree.haltTree();

      status = tree.tickRoot();

      // std::this_thread::sleep_for(std::chrono::milliseconds(50));

      // tree.tickRoot();
      //  tree.sleep(std::chrono::milliseconds(50));
      }
   

      return 0;
        }
