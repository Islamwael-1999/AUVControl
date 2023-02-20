#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from rcl_interfaces.srv import SetParameters

class VisualOdometry(Node): 
    def __init__(self):
        super().__init__('Node')
        #print('HIIIIIIIIIIIIIIIIIIIi')
        self.get_logger().info('Node init')
        camerainfo_qos = QoSProfile(
                                    depth=10, #number of messages in a buffer
                                    reliability=QoSReliabilityPolicy.BEST_EFFORT, #To make it always work at best
                                    durability=QoSDurabilityPolicy.VOLATILE)
        self.imageSubscriber = self.create_subscription(Image,"/rexrov/rexrov/camera/image_raw",self.image_receive_callback,10)
        self.imageLeftSubscriber = self.create_subscription(Image,"/rexrov/rexrov/cameraleft/image_raw",self.imageleft_receive_callback,10)
        self.imageRightSubscriber = self.create_subscription(Image,"/rexrov/rexrov/cameraright/image_raw",self.imageright_receive_callback,10)
        self.cameraSubscriber = self.create_subscription(CameraInfo,"/rexrov/rexrov/camera/camera_info",self.camera_receive_callback,camerainfo_qos)
        self.cameraLeftSubscriber = self.create_subscription(CameraInfo,"/rexrov/rexrov/cameraleft/camera_info",self.cameraleft_receive_callback,camerainfo_qos)
        self.cameraRightSubscriber = self.create_subscription(CameraInfo,"/rexrov/rexrov/cameraright/camera_info",self.cameraright_receive_callback,camerainfo_qos)
        self.setParamClient = self.create_client(SetParameters,"/rexrov/cameraleft_controller/set_parameter")
        
        #self.i = 0
        #self.flag = 0
        self.leftImage=[]
        self.rightImage = []
        self.get_logger().info('Subscriber init')
        self.bridge = CvBridge()

    
    def camera_receive_callback(self,msg):
        self.get_logger().info('Middle camera')
        self.get_logger().info('K = {}'.format((msg.k).round(3)))
        self.get_logger().info('R = {}'.format(msg.r.round(3)))
        self.get_logger().info('P = {}'.format(msg.p.round(3)))
        
    def cameraleft_receive_callback(self,msg):
        self.get_logger().info('Left camera')
        self.get_logger().info('K = {}'.format(msg.k.round(3)))
        self.get_logger().info('R = {}'.format(msg.r.round(3)))
        self.get_logger().info('P = {}'.format(msg.p.round(3)))
    def cameraright_receive_callback(self,msg):
        self.get_logger().info('Right camera')
        self.get_logger().info('K = {}'.format(msg.k.round(3)))
        self.get_logger().info('R = {}'.format(msg.r.round(3)))
        self.get_logger().info('P = {}'.format(msg.p.round(3)))

    def image_receive_callback(self,img):
        cv_img = self.bridge.imgmsg_to_cv2(img,desired_encoding="bgr8")
        cv2.imshow("middle",cv_img)
        #self.out.write(cv_img)
        #self.i+=1
        key = cv2.waitKey(1)

        
        

    def imageleft_receive_callback(self,img):
        cv_img = self.bridge.imgmsg_to_cv2(img,desired_encoding="bgr8")
        self.leftImage = cv_img
        cv2.imshow("left",cv_img)
        cv2.waitKey(3)

    def imageright_receive_callback(self,img):
        cv_img = self.bridge.imgmsg_to_cv2(img,desired_encoding="bgr8")
        self.rightImage = cv_img
        cv2.imshow("right",cv_img)
        cv2.waitKey(3)
 

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometry()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()