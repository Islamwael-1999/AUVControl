#!/usr/bin/env python
import os, sys
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
import time
from pathlib import Path
import torch
import torch.backends.cudnn as cudnn

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())


from models.experimental import attempt_load
from utils.augmentations import letterbox
from utils.general import check_img_size, non_max_suppression
from utils.plots import Annotator, colors
from utils.torch_utils import select_device

bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        weights='/home/zeiad/sensorfusion_ros2_ws/src/lqrpy/lqrpy/best.pt'  # model.pt path(s)
        self.imgsz=640  # inference size (pixels)
        self.conf_thres=0.25  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=1000  # maximum detections per image
        self.classes=None  # filter by class: --class 0, or --class 0 2 3
        self.agnostic_nms=False  # class-agnostic NMS
        self.line_thickness=3  # bounding box thickness (pixels)
        self.stride = 32
        self.swift_camera_topic="/swift/zed_rgb/image_raw"
        self.rexrov_camera_topic="/rexrov/rexrov/camera/image_raw"
        device_num=''  # cuda device, i.e. 0 or 0,1,2,3 or cpu

        # Initialize
        self.device = select_device(device_num)

        # Load model
        self.model = attempt_load(weights, device=self.device)  # load FP32 model
        stride = int(self.model.stride.max())  # model stride
        imgsz = check_img_size(self.imgsz, s=stride)  # check image size
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names

        # Dataloader
        cudnn.benchmark = True  # set True to speed up constant image size inference

        # Run inference
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, imgsz, imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once

        self.subscription = self.create_subscription(
            Image,
            self.rexrov_camera_topic,
            self.camera_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.position_publisher_ =self.create_publisher(Odometry,"Gate_position",100)


    def camera_callback(self, data):
        t0 = time.time()
        img = bridge.imgmsg_to_cv2(data, "bgr8")

        # check for common shapes
        s = np.stack([letterbox(x, self.imgsz, stride=self.stride)[0].shape for x in img], 0)  # shapes
        self.rect = np.unique(s, axis=0).shape[0] == 1  # rect inference if all shapes equal
        if not self.rect:
            print('WARNING: Different stream shapes detected. For optimal performance supply similarly-shaped streams.')

        # Letterbox
        img0 = img.copy()
        img = img[np.newaxis, :, :, :]        

        # Stack
        img = np.stack(img, 0)

        # Convert
        img = img[..., ::-1].transpose((0, 3, 1, 2))  # BGR to RGB, BHWC to BCHW
        img = np.ascontiguousarray(img)

        img = torch.from_numpy(img).to(self.device)
        img = img.float()
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        pred = self.model(img)[0]

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            s = f'{i}: '
            s += '%gx%g ' % img.shape[2:]  # print string
            im0 = img0
            annotator = Annotator(im0, line_width=self.line_thickness, example=str(self.names))
            if len(det):
                # Print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string
                
                for *xyxy, conf, cls in reversed(det):
         
                    
               
                
                    
                 

                    c = int(cls)  # integer class
                    if self.names[c]=="Gate" and conf.item()>=0.6:
                       x1=xyxy[0].item()
                       y1=xyxy[1].item()
                       x2=xyxy[2].item()
                       y2=xyxy[3].item()
                       position_msg = Odometry()
                       position_msg.pose.pose._position.x=x1
                       position_msg.pose.pose._position.y=y1
                       position_msg.twist.twist.linear.x=x2
                       position_msg.twist.twist.linear.y=y2

                       position_msg.pose.pose._position.z=float(1)
                       self.position_publisher_.publish(position_msg)
                    else:
                       position_msg = Odometry()
                       position_msg.pose.pose._position.z=float(0)
                       self.position_publisher_.publish(position_msg)
                                    


                    print("name:"+str(self.names[c])+"conf :"+str(conf.item()))

                    label = f'{self.names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))


        cv2.imshow("IMAGE", img0)
        cv2.waitKey(1)    


def main(args=None):
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
