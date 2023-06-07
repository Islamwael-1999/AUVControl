

#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from trajectory_planning.Herm import TrajectoryPlanner
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
import transformations
from gazebo_msgs.msg import ModelStates

from functools import partial
import time
from math import pi
class CubicPath(Node):


    def __init__(self):

        super().__init__("cubicPath")

        self.counter_ = 0

        self.get_logger().info("Cubic Path Started")

        self.currentState = [0,0,0,0,0,0,0,0,0,0,0,0]
        self.CurrentStateReceiver = self.create_subscription(
            ModelStates, '/gazebo/model_states', self.stateReceiveCallBack, 10)       #Extract Ground Truth (ModelStates) Message Subscriber
        # self.sensorFusionStateReceiver = self.create_subscription(
        #     Odometry, '/kalmen_filter/state', self.stateReceiveCallBack, 10)            #Estimated State (Odometry) Message Subscriber 
        # self.initWayPointsService = self.create_service(MultiArrayLayout,"/trajectory/initwaypoints",self.initWayPointsServiceCallBack)
        self.sendTargetStateClient = self.create_client(srv_type=SetModelState,srv_name="/control/set_next_state")
        while not self.sendTargetStateClient.wait_for_service(1.0):
            self.get_logger().warn("Waiting For Control Module...")
                #major timestamps
        times = [0.0,90.0,160.0,200.0,250.0,300.0]
        #major milestones positions
        state = [[0,0,-1,0,0,0],[3,3,-2,0,0,pi/4],[5,5,-2,0,0,pi/2],[3,7,-2,0,0,3*pi/4],[-3,3,-2,0,0,5*pi/4],[0,0,-2,0,0,2*pi]]
        #major milestones velocities
        velc= [[0,0,0,0,0,0],[0.1,0.1,0,0,0,0],[-0.1,0.1,0,0,0,0],[-0.1,-0.1,0,0,0,0],[0.1,-0.1,0,0,0,0],[0,0,0,0,0,0]]
        self.samplingFrequency=20
        self.traj=TrajectoryPlanner(times=times,positions=state,velocities=velc)
        self.path=[]
        for i in range(0,int(self.samplingFrequency*times[-1]),1):
            self.path.append(self.traj.eval_state(i/self.samplingFrequency))
        self.publishNextTargetState()
    def stateReceiveCallBack(self, state):
            
        self.currentState = self.ExtractGroundTruth(state)  # Extract GroundTruth
     #   self.currentState = self.ExtractOdometryState(state)
    def ExtractOdometryState(self, state):
        # Extracting Global State
        # Position:
        
        # pos = state.pose[3]        ModelStates Message      
        # position = pos.position
        position = state.pose.pose.position     # Odemetry Message
        xpos, ypos, zpos = position.x, position.y, position.z

        # Orientation
        # quat = pos.orientation       ModelStates Message 
        quat = state.pose.pose.orientation    # Odemetry Message

        (roll, pitch, yaw) = transformations.euler_from_quaternion(
            [quat.w, quat.x, quat.y, quat.z])

        # Linear Velocities
        # twist = state.twist[3]        ModelStates Message
        # linear = twist.linear
        
        linear = state.twist.twist.linear      #Odemetry Message
        vx, vy, vz = linear.x, linear.y, linear.z

        # Angular Velocities
    
        # ang = twist.angular           ModelStates Message
        ang = state.twist.twist.angular         # Odometry Message
        rollSpeed, pithSpeed, YawSpeed = ang.x, ang.y, ang.z   

        return [xpos, ypos, zpos, roll, pitch, yaw, vx, vy, vz, rollSpeed, pithSpeed, YawSpeed]
    def ExtractGroundTruth(self,state):
        # Extracting Global State
        # Position:
        
        pos = state.pose[3]        #ModelStates Message      
        position = pos.position
        xpos, ypos, zpos = position.x, position.y, position.z

        # Orientation
        quat = pos.orientation     #  ModelStates Message 

        (roll, pitch, yaw) = transformations.euler_from_quaternion(
            [quat.w, quat.x, quat.y, quat.z])

        # Linear Velocities
        twist = state.twist[3]        #ModelStates Message
        linear = twist.linear
        
        vx, vy, vz = linear.x, linear.y, linear.z

        # Angular Velocities
    
        ang = twist.angular          # ModelStates Message
        rollSpeed, pithSpeed, YawSpeed = ang.x, ang.y, ang.z   

        return [xpos, ypos, zpos, roll, pitch, yaw, vx, vy, vz, rollSpeed, pithSpeed, YawSpeed]
    def sendNextTarget(self,pathPoint):
        request = SetModelState.Request()

        request.model_state.pose.position.x = float(pathPoint[0])
        request.model_state.pose.position.y = float(pathPoint[1])
        request.model_state.pose.position.z = float(pathPoint[2])
        print("X:",request.model_state.pose.position.x)
        print("Y:",request.model_state.pose.position.y)
        print("Z:",request.model_state.pose.position.z)
        w,x,y,z= transformations.quaternion_from_euler(pathPoint[3],pathPoint[4],pathPoint[5])
        print("Roll:",pathPoint[3])
        print("Pitch:",pathPoint[4])
        print("Yaw:",pathPoint[5])
        request.model_state.pose.orientation.x = x
        request.model_state.pose.orientation.y = y
        request.model_state.pose.orientation.z = z
        request.model_state.pose.orientation.w = w
        

        request.model_state.twist.linear.x = float(pathPoint[6] )  
        request.model_state.twist.linear.y = float(pathPoint[7])
        request.model_state.twist.linear.z = float(pathPoint[8])
        print("velX:",request.model_state.twist.linear.x)
        print("velY:",request.model_state.twist.linear.y)
        print("velZ:",request.model_state.twist.linear.z)
        request.model_state.twist.angular.x =float(pathPoint[9])
        request.model_state.twist.angular.y =float(pathPoint[10])
        request.model_state.twist.angular.z =float(pathPoint[11])
        print("velRoll:",request.model_state.twist.angular.x)
        print("velPitch:",request.model_state.twist.angular.y)
        print("velYaw:",request.model_state.twist.angular.z)

    
        future = self.sendTargetStateClient.call_async(request=request)
        # future.add_done_callback(
        #     partial(self.sendTargetStateClient.callBack_to_Response_Recieve)
        # )
    def initWayPointsServiceCallBack(self,msg):
        print(msg)
    def publishNextTargetState(self):
        i=0
        self.sendNextTarget(self.path[0])
        t=0
        while i<len(self.path):
            self.sendNextTarget(self.path[i])
            i+=1
            time.sleep(1/self.samplingFrequency)
def main(args=None):

    rclpy.init(args=args)

    node = CubicPath()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":

    main()

