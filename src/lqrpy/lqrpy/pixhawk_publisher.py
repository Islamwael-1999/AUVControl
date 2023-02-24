#!/user/bin/env python3
import rclpy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import FluidPressure
from lqrpy.Pixhawk import Pixhawk
from rclpy.node import Node

class PixhawkDataNode(Node):
  
  def __init__(self):
    super().__init__("pixhawk_data_node")
    self.publisher_ =self.create_publisher(Imu,"IMU_reading",100)
    self.create_timer(0.0001,self.publish_IMU_data)
    self.get_logger().info("Pixhawk data is ready")
    self.pixhawk=Pixhawk()

  def publish_IMU_data(self):
    msg = Imu
    imu_raw=self.pixhawk.sensors.getAttitude()
    print(type(imu_raw))
    print(imu_raw)
    
    


def main(args=None):
  rclpy.init(args=args)
  node = PixhawkDataNode()
  rclpy.spin(node)
  rclpy.shutdown()

if __name__ == "__main__":
  main()
