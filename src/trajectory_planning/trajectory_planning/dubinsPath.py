#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_planning.xmx import Waypoint, calculatePath
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import transformations
from gazebo_msgs.srv import SetModelState
from functools import partial

import time
class TrajectoryPlan(Node):

    def __init__(self):
        super().__init__("TrajectoryPlanner")
        self.get_logger().info("Trajectory Planner init")
        self.currentState = [0,0,0,0,0,0,0,0,0,0,0,0]
        # self.CurrentStateReceiver = self.create_subscription(
        #     ModelStates, '/gazebo/model_states', self.stateReceiveCallBack, 10)       #Extract Ground Truth (ModelStates) Message Subscriber
        self.sensorFusionStateReceiver = self.create_subscription(
            Odometry, '/kalmen_filter/state', self.stateReceiveCallBack, 10)            #Estimated State (Odometry) Message Subscriber 

        self.sendTargetStateClient = self.create_client(srv_type=SetModelState,srv_name="/control/set_next_state")
        while not self.sendTargetStateClient.wait_for_service(1.0):
            self.get_logger().warn("Waiting For Control Module...")

        self.responseReceived = False 

    # Set WayPoints
        pt1 = Waypoint(0,0,0)
        pt2 = Waypoint(10,10,100)
        pt3 = Waypoint(20,20,180)
        pt4 = Waypoint(0,0,0)

        self.Wptz = [pt1, pt2,pt3,pt4]
        
        self.path = calculatePath(self.Wptz)

        self.publishNextTargetState()

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

    def stateReceiveCallBack(self, state):
            
        # self.currentState = self.ExtractGroundTruth(state)  # Extract GroundTruth
        self.currentState = self.ExtractOdometryState(state)

    def sendNextTarget(self,pathPoint):
        request = SetModelState.Request()

        request.model_state.pose.position.x = float(pathPoint[0])
        request.model_state.pose.position.y = float(pathPoint[1])
        request.model_state.pose.position.z = -4.0

        w,x,y,z= transformations.quaternion_from_euler(0,0,pathPoint[2])
        request.model_state.pose.orientation.x = x
        request.model_state.pose.orientation.y = y
        request.model_state.pose.orientation.z = z
        request.model_state.pose.orientation.w = w
        

        request.model_state.twist.linear.x = 0.0
        request.model_state.twist.linear.y = 0.0
        request.model_state.twist.linear.z = 0.0

        request.model_state.twist.angular.x = 0.0
        request.model_state.twist.angular.y = 0.0
        request.model_state.twist.angular.z = 0.0

    
        future = self.sendTargetStateClient.call_async(request=request)
        future.add_done_callback(
            partial(self.callBack_to_Response_Recieve)
        )

    def callBack_to_Response_Recieve(self,future):
        try :
            response = future.result()
            self.responseReceived = response.success
        except Exception as e:
            self.get_logger().error("Service call Failed %r" %(e,))
        pass
    def checkError(self,pathPoint):
        x= self.currentState[0]
        y=self.currentState[1]
        yaw = self.currentState[5]

        minErrorReached = False

        if(x-pathPoint[0]<0.01):
            minErrorReached = True
        else:
            minErrorReached = False

        if(y-pathPoint[1]<0.01):
            minErrorReached = True
        else:
            minErrorReached = False

        if(yaw-pathPoint[2]<0.01):
            minErrorReached = True
        else:
            minErrorReached = False
        print(minErrorReached)
        return minErrorReached

    def publishNextTargetState(self):
        i=0

        self.sendNextTarget(self.path[0])
        
        while i<len(self.path):
            print(self.path[i])
            # if(self.checkError(self.path[i])):
            #     i+=1
            self.sendNextTarget(self.path[i])
            i+=1
            time.sleep(6)
            
            


        # while True:
        #     # if(self)
        #     pass
            



def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlan()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
