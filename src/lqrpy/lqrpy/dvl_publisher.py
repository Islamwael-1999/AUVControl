#!/user/bin/env python3
import rclpy
from rclpy.node import Node
from dvl import Dvl
from geometry_msgs.msg import TwistWithCovarianceStamped

class DVLDataNode(Node):
  
  def __init__(self):
    super().__init__("dvl_data_node")
    self.velocity_publisher_ =self.create_publisher(TwistWithCovarianceStamped,"DVL_velocity_reading",100)
    # self.publisher_ =self.create_publisher(Imu,"IMU_reading",100)
    self.create_timer(0.06,self.publish_velocity_data)
    self.get_logger().info("dvl data is ready")
    self.dvl=Dvl()

  def publish_velocity_data(self):
    velocity_raw=self.dvl.velocity
    print(type(velocity_raw))
    print(velocity_raw)
    
    


def main(args=None):
  rclpy.init(args=args)
  node = DVLDataNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == "__main__":
  main()
