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
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import threading
import time
import message_filters
from nav_msgs.msg import Odometry
import csv


class VisualOdometry(Node): 
    def __init__(self):
        super().__init__('Visual_Odometry')
        self.get_logger().info('Visual Odometry init')
        self.P = np.zeros((3,4))
        self.x = 0
        self.y = 0
        self.z = 0
        self.trajectoryArray = []
        self.image_left = []
        self.image_plus1 = []
        self.depth = []
        self.rov_name = "rexrov"
        camerainfo_qos = QoSProfile(
                                    depth=10, #number of messages in a buffer
                                    reliability=QoSReliabilityPolicy.BEST_EFFORT, #To make it always work at best
                                    durability=QoSDurabilityPolicy.VOLATILE)
        self.groundTruth = self.create_subscription(Odometry,"/"+self.rov_name+"/pose_gt",self.odometry_receive_callback,10)
        self.cameraLeftSubscriber = self.create_subscription(CameraInfo,"/"+self.rov_name+"/zed2/depth_cam_info_demo",self.camera_receive_callback,camerainfo_qos)
        #self.imageSubscriber = self.create_subscription(Image,"/"+self.rov_name+"/zed2/image_demo",self.image_receive_callback,10)
        #self.imageDepthSubscriber = self.create_subscription(Image,"/"+self.rov_name+"/zed2/depth_demo",self.imageDepth_receive_callback,10)
        self.imageSubscriber = message_filters.Subscriber(self,Image,"/"+self.rov_name+"/zed2/image_demo")
        self.imageDepthSubscriber = message_filters.Subscriber(self,Image,"/"+self.rov_name+"/zed2/depth_demo")
        ts = message_filters.TimeSynchronizer([self.imageSubscriber, self.imageDepthSubscriber], queue_size=2)
        ts.registerCallback(self.synchronized_callback)
        self.get_logger().info('Subscriber init')
        self.position_publisher_ =self.create_publisher(Odometry,"VO_position",100)
        self.bridge = CvBridge()
        x = threading.Thread(target=self.visual_odometry)
        x.start()

    def synchronized_callback(self,img,depth):
        cv_img = self.bridge.imgmsg_to_cv2(img,desired_encoding="bgr8")
        depth_img = self.bridge.imgmsg_to_cv2(depth,desired_encoding="32FC1")
        self.depth.append(np.array(depth_img, dtype = np.float))
        if len(self.image_left)==0:
            self.image_left = cv_img
            self.image_left = cv2.cvtColor(self.image_left,cv2.COLOR_BGR2GRAY)
        else:
            self.image_plus1 = cv_img
            self.image_plus1 = cv2.cvtColor(self.image_plus1,cv2.COLOR_BGR2GRAY)
            time.sleep(0.8)
           
    def odometry_receive_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z
        
    def camera_receive_callback(self,msg):
        self.P= msg.p.reshape((3,4))

    def decompose_projection_matrix(self,p):
        k,r,t, _, _ , _ , _ = cv2.decomposeProjectionMatrix(p)
        t = (t/t[3])[:3]
        return k,r,t

    def extract_features(self, image , detector, mask = None):
        if detector == 'sift':
            det = cv2.SIFT_create()
        elif detector == 'orb':
            det = cv2.ORB_create()
        elif detector == 'akaze':
            det = cv2.AKAZE_create()    

        kp , des = det.detectAndCompute(image , mask) #keypoints and descriptors

        return kp , des

    def match_features(self,des1, des2, matching='BF' , detector = "akaze" ,sort= False , k=2):
        if matching == 'BF':
            if detector == 'sift':
                matcher = cv2.BFMatcher_create(cv2.NORM_L1, crossCheck=False) #intensities and image gradient
            elif detector == 'orb':
                matcher = cv2.BFMatcher_create(cv2.NORM_HAMMING2, crossCheck=False) #binary keypoint
            elif detector == 'akaze':
                matcher = cv2.BFMatcher_create(cv2.NORM_HAMMING,crossCheck=False)
        elif matching == 'FLANN':
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
            search_params = dict(checks = 50)
            matcher = cv2.FlannBasedMatcher_create(index_params,search_params)

        if detector == 'sift' or detector == 'orb':
            matches = matcher.knnMatch(des1,des2,k=k)
        elif detector == 'akaze':
            matches = matcher.match(des1,des2)

        if sort:
            if detector == 'akaze':
                matches = sorted(matches , key =lambda x: x.distance)
            else:
                matches = sorted(matches , key =lambda x: x[0].distance)
        return matches
    
    def visualize_matches(self,image1, kp1, images2, kp2, match):
        image_matches = cv2.drawMatches(image1, kp1, images2, kp2, match, None , flags=2)
        plt.close()
        plt.figure(figsize= (16,6), dpi= 100)
        plt.imshow(image_matches)
        plt.pause(5)

    def filter_matches_distance(self,detector, matches, dist_threshold = 0.375):
        filtered_matches = []
        counting = 0
        if detector == 'akaze':
            for m in matches:
                #print(m.distance)
                if m.distance <= 30:
                    counting = counting+1
                    filtered_matches.append(m)
            print("Length of matches = "+str(len(matches)))
            print("Matches less than 25 = "+ str(counting))
            
        else:   
            for m, n in matches:
                if m.distance <= dist_threshold * n.distance:
                    counting = counting + 1
                    filtered_matches.append(m)
                
            print("Length of matches = "+str(len(matches)))
            print("Matches after filter = "+ str(counting))
        
        return filtered_matches

    def estimate_motion(self,matches, kp1, kp2, k, depth1, max_depth=200):
    
        rmat = np.eye(3)
        tvec = np.zeros((3,1))

        image1_points= np.float32([kp1[m.queryIdx].pt for m in matches])
        image2_points= np.float32([kp2[m.trainIdx].pt for m in matches])

        cx = k[0, 2]
        cy = k[1, 2]
        fx = k[0, 0]
        fy = k[1, 1]

        object_points = np.zeros((0,3))
        delete = []

        for i,(u, v) in enumerate(image1_points):
            z = depth1[0][int(round(v)), int(round(u))]

            if z > max_depth:
                delete.append(i)
                continue

            x = z * (u - cx) / fx
            y = z * (v - cy) / fy

            object_points = np.vstack([object_points, np.array([x,y,z])])
        
        image1_points = np.delete(image1_points, delete, 0)
        image2_points = np.delete(image2_points, delete, 0)
        if len(image2_points)<40 or len(object_points)<40:
            return None,None

        #_, rvec, tvec, inliers = cv2.solvePnPRansac(object_points, image2_points, k, None,useExtrinsicGuess=True,reprojectionError=5,confidence=0.95)
        _, rvec, tvec, inliers = cv2.solvePnPRansac(object_points, image2_points, k, None)
        tempx = tvec[0][0].copy()
        tempy = tvec[1][0].copy()
        tempz = tvec[2][0].copy()
        tvec[0][0] = tempz        #dont change
        tvec[1][0] = -1*tempx
        tvec[2][0] = -1*tempy        
        #print(tvec)
        rmat = cv2.Rodrigues(rvec)[0] #changes from matrix to vector

        return rmat, tvec
    def visual_odometry(self,detector = 'sift', matching = 'BF', filter_match_distance= 0.425):
        time.sleep(0.1)
        while self.P[0][0] == 0 and (self.x==0 or self.y==0 or self.z==0):
            pass
        # while self.P[0][0] == 0:
        #     pass
        new_x = float(self.x)
        new_y = float(self.y)
        new_z = float(self.z)
        T_tot = np.asarray((1,0,0,new_x,0,1,0,new_y,0,0,1,new_z,0,0,0,1),dtype = float).reshape(4,4)
        #print(T_tot)
        self.trajectoryArray.append(T_tot[:3, :])
        #print(self.P)
        k, r, t = self.decompose_projection_matrix(self.P)
        # with open('visualise1.csv', 'x', newline='') as file:
        #     writer = csv.writer(file)
        #     field = ["errorX","errorY","errorZ","estimatedX","estimatedY","estimatedZ","actualX","actualY","actualZ","PercenterrorX","PercenterrorY","PercenterrorZ","timediff"]
        #     writer.writerow(field)
        start = time.time()
        while True:
            if len(self.depth) == 0 or len(self.image_left) ==0 or len(self.image_plus1)==0:
                continue

            # cv2.imshow("img_left",self.image_left)
            #cv2.imshow("img_plus1",self.image_plus1)
            # cv2.waitKey(1)

            kp0, des0 = self.extract_features(self.image_left, detector)
            kp1, des1 = self.extract_features(self.image_plus1, detector)
            if des1 is None or des0 is None or kp0 is None or kp1 is None:
                self.depth = []
                self.image_left = []
                self.image_plus1 = []
                #print("entered first if")
                continue

            matches_unfilt = self.match_features(des0, des1, matching=matching, detector=detector)
            
            if filter_match_distance is not None:
                matches = self.filter_matches_distance(detector,matches_unfilt, filter_match_distance)
            else:
                matches = matches_unfilt
            #self.visualize_matches(self.image_left,kp0,self.image_plus1,kp1,matches)

            rmat, tvec = self.estimate_motion(matches, kp0, kp1, k, self.depth)
            if tvec is None:
                self.depth = []
                self.image_left = []
                self.image_plus1 = []
                continue
            
            # print('delta x = ' + str(tvec[0][0]))
            # print('delta y = ' + str(tvec[1][0]))
            # print('delta z = ' + str(tvec[2][0]))  
            Tmat = np.eye(4)
            Tmat[:3, :3] = rmat
            Tmat[:3, 3] = tvec.T

            T_tot = T_tot.dot(np.linalg.inv(Tmat))
            self.trajectoryArray.append(T_tot[:3, :])
            print("x = "+ str(self.trajectoryArray[len(self.trajectoryArray)-1][0,3]))
            print("y = "+ str(self.trajectoryArray[len(self.trajectoryArray)-1][1,3]))
            print("z = "+ str(self.trajectoryArray[len(self.trajectoryArray)-1][2,3]))
            end = time.time()
            diff = end - start
            errorX = abs(self.x - self.trajectoryArray[len(self.trajectoryArray)-1][0,3])
            errorY = abs(self.y - self.trajectoryArray[len(self.trajectoryArray)-1][1,3])
            errorZ = abs(self.z - self.trajectoryArray[len(self.trajectoryArray)-1][2,3])
            estimatedX = self.trajectoryArray[len(self.trajectoryArray)-1][0,3]
            estimatedY = self.trajectoryArray[len(self.trajectoryArray)-1][1,3]
            estimatedZ = self.trajectoryArray[len(self.trajectoryArray)-1][2,3]
            # with open('visualise1.csv', 'a', newline='') as file:
            #     writer = csv.writer(file)
            #     writer.writerow([errorX,errorY,errorZ,estimatedX,estimatedY,estimatedZ,self.x,self.y,self.z,(errorX/self.x)*100,(errorY/self.y)*100,(errorZ/self.z)*100,diff])
            print("len of depth array = " + str(len(self.depth)))
            print("len of matches = "+ str(len(matches)))
            #print("shape of image = " + str(self.image_left.shape))
            #print("shape of image plus 1 = " + str(self.image_plus1.shape))
            print("Error in x = " + str(abs(self.x - self.trajectoryArray[len(self.trajectoryArray)-1][0,3])))
            print("Error in y = " + str(abs(self.y - self.trajectoryArray[len(self.trajectoryArray)-1][1,3])))
            print("Error in z = " + str(abs(self.z - self.trajectoryArray[len(self.trajectoryArray)-1][2,3])))
            self.depth.pop(0)
            self.image_left = self.image_plus1
            self.image_plus1 = []
            position_msg = Odometry()
            position_msg.pose.pose._position.x=self.trajectoryArray[len(self.trajectoryArray)-1][0,3]
            position_msg.pose.pose._position.y=self.trajectoryArray[len(self.trajectoryArray)-1][1,3]
            self.position_publisher_.publish(position_msg)
            # self.depth = []
            # self.image_left = []
            # self.image_plus1 = []
            
def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometry()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()