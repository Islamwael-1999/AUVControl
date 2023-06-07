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

bool idle_forward = true;
bool idle_rotate = true;

class Robot_state  : public rclcpp::Node
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
    roll_reading  = roll;
    pitch_reading = pitch;
    yaw_reading   = yaw;

    x_reading = msg->pose[3].position.x;
    y_reading = msg->pose[3].position.y;
    z_reading = msg->pose[3].position.z;

    // std::cout << "yaw : " << yaw_reading << std::endl;
    // std::cout << "x   : " << x_reading << std::endl;
  }
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr Oriantation_subscriber_;
  std::string topic_ = "/gazebo/model_states";
  
};



class Check_Rotation : public BT::AsyncActionNode, public rclcpp::Node
{
  public:
    Check_Rotation(const std::string &name, const BT::NodeConfiguration &config) : BT::AsyncActionNode(name, config), Node("check_rotation_node")
    {
       RCLCPP_INFO(this->get_logger(), "subscriber ground truth initialized");
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
      std::cout << "rotate state:" << status() << std::endl;

      if (status() == NodeStatus::IDLE || idle_rotate == true)
      {
        idle_rotate = false;
        std::cout<< "in rotate idle" << std::endl;
        
        // Oriantation_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        //     topic_, 100, std::bind(&Check_Rotation::callbackCheckRotation, this, _1));
        // std::cout << "rotate subscriber init" << std::endl;

        // Rotate_Client_ = this->create_client<uuv_control_msgs::srv::GoTo>(service_);
        // while (!Rotate_Client_->wait_for_service(std::chrono::seconds(1)))
        // {
        //   std::cout << "waiting for service to be up...." << std::endl;
        // }
        // std::cout << "client created" << std::endl;
        // auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
        // request->waypoint.point.set__x(0);
        // request->waypoint.point.set__y(6);
        // request->waypoint.point.set__z(-20);
        // request->waypoint.set__max_forward_speed(0.2);
        // request->waypoint.set__heading_offset(0);
        // request->waypoint.set__use_fixed_heading(true);
        // Rotate_Client_->async_send_request(request);

        setStatus(NodeStatus::FAILURE);
      }
      else if (status() == NodeStatus::RUNNING)
      {

        if (yaw_reading > 5)
        {
          Oriantation_subscriber_.reset();
          setStatus(NodeStatus::SUCCESS);
          std::cout << "in rotate success" << std::endl;
        }
      }
      std::cout << "out of rotate " << std::endl;
      return status();                        
    }
    static BT::PortsList providedPorts() { return {}; }

  private:
    // void callbackCheckRotation(const gazebo_msgs::msg::ModelStates::SharedPtr msg){
    //   std::cout << "oriantation update " << std::endl;
    //   tf2::Quaternion q(
    //       msg->pose[3].orientation.x,
    //       msg->pose[3].orientation.y,
    //       msg->pose[3].orientation.z,
    //       msg->pose[3].orientation.w);
    //   tf2::Matrix3x3 m(q);
    //   double roll, pitch, yaw;
    //   m.getRPY(roll, pitch, yaw);
    //   yaw_reading = yaw;

    // } 
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr Oriantation_subscriber_;
    rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Rotate_Client_;
    std::string topic_ = "/gazebo/model_states";
    std::string service_ = "/rexrov/go_to";
};

class Check_point : public BT::AsyncActionNode , public rclcpp::Node
{
  public:
    Check_point(const std::string &name, const BT::NodeConfiguration &config) : BT::AsyncActionNode(name, config), Node("Check_point_node")
    {
        // Oriantation_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 100,
        //                                                                                    std::bind(&Check_point::callbackCheckPoint, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "subscriber ground truth initialized");
    }
    // You must override the virtual function tick()
    NodeStatus tick() override
    {

        std::cout << "forward state:" << status() << std::endl;
        if (status() == NodeStatus::IDLE || idle_forward == true)
        {
        idle_forward = false;
        // std::cout << "in forward idle" << std::endl;
        // Oriantation_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
        //     topic_, 100, std::bind(&Check_point::callbackCheckPoint, this, _1));
        // std::cout << "forward subscriber init" << std::endl;

        // Forward_Client_ = this->create_client<uuv_control_msgs::srv::GoTo>(service_);
        // while (!Forward_Client_->wait_for_service(std::chrono::seconds(1)))
        // {
        //   std::cout << "waiting for service to be up...." << std::endl;
        // }
        // std::cout << "client created" << std::endl;
        // auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
        // request->waypoint.point.set__x(6);
        // request->waypoint.point.set__y(6);
        // request->waypoint.point.set__z(-20);
        // request->waypoint.set__max_forward_speed(0.2);
        // request->waypoint.set__heading_offset(0);
        // request->waypoint.set__use_fixed_heading(true);
        // Forward_Client_->async_send_request(request);

        setStatus(NodeStatus::FAILURE);
        }
        else if (status() == NodeStatus::RUNNING)
        {

        if (x_reading > 5)
        {
          Oriantation_subscriber_.reset();
          setStatus(NodeStatus::SUCCESS);
          std::cout << "in forward success" << std::endl;
        }
        }
        return status();
    }
    static BT::PortsList providedPorts() { return {}; }
  private:
    // void callbackCheckPoint(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    // {
    //     std::cout << "pose update " << std::endl;
    //     x_reading = msg->pose[3].position.x;
    // }
    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr Oriantation_subscriber_;
    std::string topic_ = "/gazebo/model_states";
    std::string service_ = "/rexrov/go_to";
};

// class Task2 : public BT::SyncActionNode
// {
//   public:
//     Task2(const std::string& name) : BT::SyncActionNode(name, {})
//     {
//     }
//     // You must override the virtual function tick()
//     NodeStatus tick() override
//     {
//         std::cout << "Task2: " << this->name() << std::endl;
//         return BT::NodeStatus::FAILURE;
//     }
// };



// class Move_Backward : public BT::SyncActionNode, public rclcpp::Node
// {
//   public:
//     double max_speed = 0.2;
//     double current_x;
//     double current_y;
//     double current_z;
//     double current_heading;
//     double target_x;
//     double target_y;
//     double target_z;
//     double target_heading;
//     bool fist_entry = true;
//     bool poseReady = false;
//     Move_Backward(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config), Node("movebackward")
//     {
//         Oriantation_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 100,
//                                                                                            std::bind(&Move_Backward::callbackCheckPosition, this, std::placeholders::_1));
//         Backward_Client_ = this->create_client<uuv_control_msgs::srv::GoTo>("/rexrov/go_to");
//         while (!Backward_Client_->wait_for_service(std::chrono::seconds(1)))
//         {
//           RCLCPP_WARN(this->get_logger(), "waiting for service to be up....");
//         }
//         RCLCPP_INFO(this->get_logger(), "client created");
//     }
//     NodeStatus tick() override
//     {
//         std::cout << "Move_Backward: " << this->name() << std::endl;
//         if (fist_entry == true && poseReady == true)
//         {
//           fist_entry = false;
//           target_x = current_x - 2;
//           target_y = current_y;
//           target_z = current_z;
//           target_heading = current_heading;
//           send_waypoint(target_x,
//                         target_y,
//                         target_z,
//                         max_speed,
//                         target_heading);
//         }
//         if (current_x >= target_x - 0.2 && current_x <= target_x + 0.2)
//         {
//           fist_entry = true;
//           return BT::NodeStatus::SUCCESS;
//         }
//         else
//         {
//           return BT::NodeStatus::FAILURE;
//         }
//     }
//     static BT::PortsList providedPorts() { return {}; }
//   private:
//     void callbackCheckPosition(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
//     {
//         poseReady = true;
//         tf2::Quaternion q(
//             msg->pose[3].orientation.x,
//             msg->pose[3].orientation.y,
//             msg->pose[3].orientation.z,
//             msg->pose[3].orientation.w);
//         tf2::Matrix3x3 m(q);
//         double roll, pitch, yaw;
//         m.getRPY(roll, pitch, yaw);
//         current_x = msg->pose[3].position.x;
//         current_y = msg->pose[3].position.y;
//         current_z = msg->pose[3].position.z;
//         current_heading = yaw;
//     }
//     void send_waypoint(float x, float y, float z, float max_speed, float heading)
//     {
//         auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
//         request->waypoint.point.set__x(x);
//         request->waypoint.point.set__y(y);
//         request->waypoint.point.set__z(z);
//         request->waypoint.set__max_forward_speed(max_speed);
//         request->waypoint.set__heading_offset(heading);
//         request->waypoint.set__use_fixed_heading(true);
//         Backward_Client_->async_send_request(request);
//     }
//     rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr Oriantation_subscriber_;
//     rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Backward_Client_;
// };

// class Move_Forward : public BT::SyncActionNode, public rclcpp::Node
// {
//    public:
//      Move_Forward(const std::string &name, const BT::NodeConfiguration &config) : BT::SyncActionNode(name, config), Node("moveForward")
//      {
//         // Oriantation_subscriber_ = this->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 100,
//         //                                                                                    std::bind(&Move_Forward::callbackCheckPosition, this, std::placeholders::_1));
//         Forward_Client_ = this->create_client<uuv_control_msgs::srv::GoTo>("/rexrov/go_to");
//         while (!Forward_Client_->wait_for_service(std::chrono::seconds(1)))
//         {
//           RCLCPP_WARN(this->get_logger(), "waiting for service to be up....");
//        }
//        RCLCPP_INFO(this->get_logger(), "client created");
//      }
//      NodeStatus tick() override
//      {
//        std::cout << "Move_Forward: " << this->name() << std::endl;
//        if(fist_entry == true && poseReady == true){
//           fist_entry=false;
//           target_x = current_x + 2;
//           target_y = current_y;
//           target_z = current_z;
//           target_heading = current_heading;
//           send_waypoint(target_x,
//                         target_y,
//                         target_z,
//                         max_speed,
//                         target_heading);       
//        }
//        if (current_x >= target_x - 0.2 && current_x <= target_x + 0.2){
//           fist_entry=true;
//           return BT::NodeStatus::SUCCESS;
//        }
//        else{
//           return BT::NodeStatus::FAILURE;
//        } 
//      }
//      static BT::PortsList providedPorts(){return{};}
//    private:
//      void callbackCheckPosition(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
//      {
//       poseReady=true;
//        tf2::Quaternion q(
//            msg->pose[3].orientation.x,
//            msg->pose[3].orientation.y,
//            msg->pose[3].orientation.z,
//            msg->pose[3].orientation.w);
//        tf2::Matrix3x3 m(q);
//        double roll, pitch, yaw;
//        m.getRPY(roll, pitch, yaw);
//        current_x = msg->pose[3].position.x;
//        current_y = msg->pose[3].position.y;
//        current_z = msg->pose[3].position.z;
//        current_heading = yaw;
//      }
//       void send_waypoint(float x, float y, float z, float max_speed, float heading)
//      {
//        auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
//        request->waypoint.point.set__x(x);
//        request->waypoint.point.set__y(y);
//        request->waypoint.point.set__z(z);
//        request->waypoint.set__max_forward_speed(max_speed);
//        request->waypoint.set__heading_offset(heading);
//        request->waypoint.set__use_fixed_heading(true);
//        Forward_Client_->async_send_request(request);
//        }
//      rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr Oriantation_subscriber_;
//      rclcpp::Client<uuv_control_msgs::srv::GoTo>::SharedPtr Forward_Client_;
// };

static const char *xml_text_medium = R"(

 <root main_tree_to_execute="BehaviorTree">

    <BehaviorTree ID="BehaviorTree">
        <Fallback name="root">
            <ReactiveSequence name="move_after_rotation">
                <Action ID="Check_Rotation"/>
                <Action ID="Check_point"/>
            </ReactiveSequence>
        </Fallback>
    </BehaviorTree>
 
</root>
 )";

int main(int argc, char **argv)
{
     rclcpp::init(argc,argv);
// auto nh = std::make_shared<rclcpp::Node>("sleep_client");
     BehaviorTreeFactory factory;

     factory.registerNodeType<Check_Rotation>("Check_Rotation");
     factory.registerNodeType<Check_point>("Check_point");
     //  factory.registerNodeType<Move_Backward>("Move_Backward");
     //  factory.registerNodeType<Move_Forward>("Move_Forward");

     std::cout << "\n------------ BUILDING A NEW TREE ------------" << std::endl;

     auto tree = factory.createTreeFromText(xml_text_medium);
     std::cout <<"\n------------ set ZMQ Publisher   ------------\n"<< std::endl;

     unsigned max_msg_per_second = 25;
     unsigned publisher_port = 1666;
     unsigned server_port = 1667;
     PublisherZMQ publisher_zmq(tree, max_msg_per_second, publisher_port, server_port);
     NodeStatus status = NodeStatus::FAILURE;
     BT::NodeConfiguration con = {};
     
     auto lc_check_rotation = std::make_shared<Check_Rotation>("Check_Rotation",con);
     auto lc_check_point = std::make_shared<Check_point>("Check_point", con);
     auto node = std::make_shared<Robot_state>();

      // auto lc_Move_backward = std::make_shared<Move_Backward>("Move_Backward", con);
      // auto lc_Move_forward = std::make_shared<Move_Forward>("Move_Forward",con);

     while(status != BT::NodeStatus::SUCCESS){
       rclcpp::spin_some(lc_check_rotation);
       rclcpp::spin_some(lc_check_point);
      //  rclcpp::spin_some(node);
      //  std::cout << "\n------------ in while   ------------\n"<< std::endl;
       //  rclcpp::spin_some(lc_Move_forward);
       //  rclcpp::spin_some(lc_Move_backward);

       status = tree.rootNode()->executeTick();

       //  tree.sleep(std::chrono::milliseconds(50));
      
     }
     
     
     return 0;
  }
  
