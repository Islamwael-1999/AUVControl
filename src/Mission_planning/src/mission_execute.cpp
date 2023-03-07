#include "rclcpp/rclcpp.hpp"
#include <uuv_control_msgs/srv/go_to.hpp>
#include "behaviortree_cpp_v3/bt_factory.h"
#include "gazebo_msgs/msg/model_states.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>

#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
using namespace BT;
class MissionExecute : public rclcpp::Node
{
  public:
    MissionExecute() : Node("mission_execute"){}
        void execute() {
          create_behavior_tree();
     
          const auto timer_period = std::chrono::milliseconds(500);
          timer_ = this->create_wall_timer(
              timer_period,
              std::bind(&MissionExecute::update_behavior_tree, this));
          rclcpp::spin(shared_from_this());
          rclcpp::shutdown();
        }
        void create_behavior_tree(){
        BehaviorTreeFactory factory;

        // factory.registerNodeType<Check_Rotate>("Check_Rotate");
        // factory.registerNodeType<Move_Forward>("Move_Forward");

        tree_ = factory.createTreeFromText(xml_text_medium);
        unsigned max_msg_per_second = 25;
        unsigned publisher_port = 1666;
        unsigned server_port = 1667;
        PublisherZMQ publisher_zmq(tree_, max_msg_per_second, publisher_port, server_port);
        
        }
        void update_behavior_tree()
        {
          // Tick the behavior tree.
          NodeStatus tree_status = tree_.tickRoot();
          if (tree_status == NodeStatus::RUNNING)
          {
            return;
          }

          // Cancel the timer if we hit a terminal state.
          if (tree_status == NodeStatus::SUCCESS)
          {
            RCLCPP_INFO(this->get_logger(), "Finished with status SUCCESS");
            timer_->cancel();
          }
          else if (tree_status == NodeStatus::FAILURE)
          {
            RCLCPP_INFO(this->get_logger(), "Finished with status FAILURE");
            timer_->cancel();
          }
        }
        rclcpp::TimerBase::SharedPtr timer_;
        Tree tree_;
        const char *xml_text_medium = R"(
            <root main_tree_to_execute="BehaviorTree">
                <BehaviorTree ID="BehaviorTree">
                    <Fallback name="root">
                        <Sequence name="move_after_rotation">
                            <Action ID="Check_Rotate"/>
                            <Action ID="Move_Forward"/>
                        </Sequence>
                    </Fallback>
                </BehaviorTree>
            </root>
            )";
};

class Move_Forward : public SyncActionNode
{
  public:
    rclcpp::Node::SharedPtr node_ptr_;
    Move_Forward(const std::string &name, const NodeConfiguration &config,
                 rclcpp::Node::SharedPtr node_ptr) : SyncActionNode(name, config), node_ptr_{node_ptr} {}
    //   BT::NodeStatus onStart() override
    //   {
    //         auto Backward_Client_ = node_ptr_->create_client<uuv_control_msgs::srv::GoTo>("/rexrov/go_to");
    //         auto Oriantation_subscriber_ = node_ptr_->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 100,
    //                                                                                                      std::bind(&Move_Forward::callbackCheckPosition, this, std::placeholders::_1));
    //         auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
    //         request->waypoint.point.set__x(5);
    //         request->waypoint.point.set__y(0);
    //         request->waypoint.point.set__z(0);
    //         request->waypoint.set__max_forward_speed(0.2);
    //         request->waypoint.set__heading_offset(0.5);
    //         request->waypoint.set__use_fixed_heading(true);
    //         Backward_Client_->async_send_request(request);
    //         return BT::NodeStatus::RUNNING;
    //  }
    //   BT::NodeStatus onRunning() override
    //   {
    //         // If there is a result, we can check the status of the action directly.
    //         // Otherwise, the action is still running.
    //         if (check_done())
    //           {
    //             std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
    //             return BT::NodeStatus::SUCCESS;
    //           }

    //         else
    //         {
    //           return BT::NodeStatus::RUNNING;
    //         }
    //   }
    void callbackCheckPosition(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
    {
                  tf2::Quaternion q(
                      msg->pose[3].orientation.x,
                      msg->pose[3].orientation.y,
                      msg->pose[3].orientation.z,
                      msg->pose[3].orientation.w);
                  tf2::Matrix3x3 m(q);
                  double roll, pitch, yaw;
                  m.getRPY(roll, pitch, yaw);
                  current_x = msg->pose[3].position.x;
                  current_y = msg->pose[3].position.y;
                  current_z = msg->pose[3].position.z;
                  current_heading = yaw;
    }
    bool check_done(){
                  if (current_x > 5)
                  {
            return true;
                  }
                  else return false;
    } 
    double current_x;
    double current_y ; 
    double current_z ; 
    double current_heading ;
    bool done = false;
    };

class Check_Rotate : public SyncActionNode
{
    public:
      rclcpp::Node::SharedPtr node_ptr_;
      Check_Rotate(const std::string &name, const NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node_ptr) : SyncActionNode(name, config), node_ptr_{node_ptr} {}
      // BT::NodeStatus onStart() override
      // {
      //         auto Backward_Client_ = node_ptr_->create_client<uuv_control_msgs::srv::GoTo>("/rexrov/go_to");
      //         auto Oriantation_subscriber_ = node_ptr_->create_subscription<gazebo_msgs::msg::ModelStates>("/gazebo/model_states", 100,
      //                                                                                                       std::bind(&Check_Rotate::callbackCheckPosition, this, std::placeholders::_1));
      //         auto request = std::make_shared<uuv_control_msgs::srv::GoTo::Request>();
      //                 request->waypoint.point.set__x(0);
      //                 request->waypoint.point.set__y(0);
      //                 request->waypoint.point.set__z(0);
      //                 request->waypoint.set__max_forward_speed(0.2);
      //                 request->waypoint.set__heading_offset(0.5);
      //                 request->waypoint.set__use_fixed_heading(false);
      //                 Backward_Client_->async_send_request(request);
      //                 return BT::NodeStatus::RUNNING;
      // }
      // BT::NodeStatus onRunning() override
      // {
      //         // If there is a result, we can check the status of the action directly.
      //         // Otherwise, the action is still running.
      //         if (true)
      //         {
      //   std::cout << "[" << this->name() << "] Failed to reach goal" << std::endl;
      //   return BT::NodeStatus::SUCCESS;
      //         }

      //         else
      //         {
      //   return BT::NodeStatus::RUNNING;
      //         }
      // }
      // void callbackCheckPosition(const gazebo_msgs::msg::ModelStates::SharedPtr msg)
      // {
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
      // }

      bool check_done()
      {
              if (current_heading > 0.5)
              {
        return true;
              }
              else
        return false;
      }
      double current_x;
      double current_y;
      double current_z;
      double current_heading;
      bool done = false;
};



int main(int argc, char **argv)
{

        rclcpp::init(argc, argv);
        auto node = std::make_shared<MissionExecute>();


          
        node->execute();
        return 0;
  }
