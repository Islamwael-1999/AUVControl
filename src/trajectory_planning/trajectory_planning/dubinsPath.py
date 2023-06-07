#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
import transformations
from gazebo_msgs.srv import SetModelState
from ruckig import InputParameter, Ruckig, Result, OutputParameter
import time
from copy import copy
import math

from auv_interfaces.srv import Waypoint

SENSOR_FUSION_STATE = False


class TrajectoryPlan(Node):

    def __init__(self):
        super().__init__("TrajectoryPlanner")
        self.get_logger().info("Trajectory Planner init")
        self.currentState = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        if not SENSOR_FUSION_STATE:
            self.CurrentStateReceiver = self.create_subscription(
                ModelStates, '/gazebo/model_states', self.stateReceiveCallBack, 10)  # Extract Ground Truth (ModelStates) Message Subscriber
        else:
            self.sensorFusionStateReceiver = self.create_subscription(
                Odometry, '/kalmen_filter/state', self.stateReceiveCallBack, 10)  # Estimated State (Odometry) Message Subscriber

        self.WayPoint = self.create_service(
            Waypoint, "/setWayPoint", self.setNextWayPointCallBack)

        self.setnextAngle = self.create_service(
            Waypoint, "/setAngle", self.setAngleCallBack)

        self.sendTargetStateClient = self.create_client(
            srv_type=SetModelState, srv_name="/control/set_next_state")
        while not self.sendTargetStateClient.wait_for_service(1.0):
            self.get_logger().warn("Waiting For Control Module...")

        self.responseReceived = False
        self.wayPointReceived = False
        self.setAngle = False

    def setNextWayPointCallBack(self,request,response):
        self.wayPointReceived = True
        self.setAngle = False
        self.targetX = request.point.x
        self.targetY = request.point.y
        self.targetZ = request.point.z
        self.targetAngle = request.point.yaw
        self.isStreeing = request.point.steering

        
        self.otg = Ruckig(3, 2)  # DoFs, control cycle , Max Number of WayPoint
        self.out = OutputParameter(3) # DoFs, Max Number of WayPoint
        self.inp = InputParameter(3)

        self.mapModelStatetoRuckigInput()
        self.inp.current_acceleration = [0.0, 0.0, 0.0]

        self.inp.target_position = [self.targetX, self.targetY, self.targetZ]
        self.inp.target_velocity = [0.0, 0.0, 0.0]
        self.inp.target_acceleration = [0.0, 0.0, 0.0]

        self.inp.max_velocity = [0.2, 0.2, 0.2]
        self.inp.max_acceleration = [0.2, 0.2, 0.2]
        self.inp.max_jerk = [6, 10, 20]

    # Set different constraints for negative direction
        self.inp.min_velocity = [-0.2, -0.2, -0.2]
        self.inp.min_acceleration = [-0.2, -0.2, -0.2]

        response.response = True
        return response

    def setAngleCallBack(self,request,response):
        self.setAngle = True
        self.targetAngle = request.point.yaw
        
        self.targetX = self.currentState[0]
        self.targetY = self.currentState[1]
        self.targetZ = self.currentState[2]
        
        response.response = True
        return response

        # self.otg = Ruckig(3, 0.5, 2)  # DoFs, control cycle , Max Number of WayPoint
        # self.out = OutputParameter(3, 2) # DoFs, Max Number of WayPoint
        # self.inp.intermediate_positions = [
        #     [4, -3, -3],
            
            
        #     [7, -8, -7]
        # ]
        # self.inp.interrupt_calculation_duration = 500  # [µs]

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

        linear = state.twist.twist.linear  # Odemetry Message
        vx, vy, vz = linear.x, linear.y, linear.z

        # Angular Velocities

        # ang = twist.angular           ModelStates Message
        ang = state.twist.twist.angular         # Odometry Message
        rollSpeed, pithSpeed, YawSpeed = ang.x, ang.y, ang.z

        return [xpos, ypos, zpos, roll, pitch, yaw, vx, vy, vz, rollSpeed, pithSpeed, YawSpeed]

    def ExtractGroundTruth(self, state):
        # Extracting Global State
        # Position:

        pos = state.pose[3]  # ModelStates Message
        position = pos.position
        xpos, ypos, zpos = position.x, position.y, position.z

        # Orientation
        quat = pos.orientation  # ModelStates Message

        (roll, pitch, yaw) = transformations.euler_from_quaternion(
            [quat.w, quat.x, quat.y, quat.z])

        # Linear Velocities
        twist = state.twist[3]  # ModelStates Message
        linear = twist.linear

        vx, vy, vz = linear.x, linear.y, linear.z

        # Angular Velocities

        ang = twist.angular          # ModelStates Message
        rollSpeed, pithSpeed, YawSpeed = ang.x, ang.y, ang.z

        return [xpos, ypos, zpos, roll, pitch, yaw, vx, vy, vz, rollSpeed, pithSpeed, YawSpeed]

    def mapModelStatetoRuckigInput(self):
        if not self.setAngle:
            self.inp.current_position = [
                self.currentState[0], self.currentState[1], self.currentState[2]]
            self.inp.current_velocity = [
                self.currentState[6], self.currentState[7], self.currentState[8]]
        else:
            self.inp2.current_position = [self.currentState[5]]

    def calculateError(self, currentState, nextWayPoint):
        errorX = currentState[0]-nextWayPoint.new_position[0]
        errorY = currentState[1]-nextWayPoint.new_position[1]
        errorZ = currentState[2]-nextWayPoint.new_position[2]
        error = math.sqrt(errorX**2+errorY**2+errorZ**2)
        print('\nError:', error, '\n')
        if error <= 0.09:
            return True
        else:
            return False

    def stateReceiveCallBack(self, state):
        if self.wayPointReceived or self.setAngle:
            if not SENSOR_FUSION_STATE:
                self.currentState = self.ExtractGroundTruth(state)
            else:
                self.currentState = self.ExtractOdometryState(state)

            if not self.setAngle:
                self.mapModelStatetoRuckigInput()
                res = self.otg.update(self.inp, self.out)
                x = self.out.new_calculation
                print('\t'.join([f'{self.out.time:0.3f}'] +
                    [f'{p:0.3f}' for p in self.out.new_position]))
                self.sendNextTarget(self.out)
                print(self.inp)
                self.out.pass_to_input(self.inp)
                if(self.calculateError(self.currentState, self.out)):
                    self.setAngle = True
                    # self.setTargetAngle(self.targetAngle)
            else:
                print('\nSetting Angle\n')
                self.setTargetAngle(self.targetAngle)
                self.mapModelStatetoRuckigInput()
                res = self.otg2.update(self.inp2, self.out2)
                # print('\t'.join([f'{self.out2.time:0.3f}'] +'Target Yaw'+ [f'{p:0.3f}' for p in self.out2.new_position]))
                self.holdState[5] = self.out2.new_position[0]
                self.holdState[11] = self.out2.new_velocity[0]
                self.sendNextAngle(self.holdState)
                print('YAW', self.out2)

    def checkDir(self, Target, current):
        if current > 0:
            if current < math.pi/2:
                return Target
            else:
                if Target > 0:
                    return Target
                else:
                    return Target + 2*math.pi
        elif current < 0:
            if current > -math.pi/2:
                return Target
            else:
                if Target < 0:
                    return Target
                else:
                    return Target - 2*math.pi

    def setTargetAngle(self, angle):
        self.holdState = self.currentState
        self.holdState[0] = self.targetX
        self.holdState[1] = self.targetY
        self.holdState[2] = self.targetZ

        self.otg2 = Ruckig(1, 2)  # DoFs, control cycle
        self.out2 = OutputParameter(1)
        self.inp2 = InputParameter(1)

        target = self.checkDir(angle, self.currentState[5])
        self.inp2.target_position = [target]
        self.inp2.target_velocity = [0.0]
        self.inp2.target_acceleration = [0.0]

        self.inp2.max_velocity = [0.5]
        self.inp2.max_acceleration = [0.1]
        self.inp2.max_jerk = [0.05]

        self.inp2.min_velocity = [-0.5]
        self.inp2.min_acceleration = [-0.1]

    def sendNextTarget(self, pathPoint):
        request = SetModelState.Request()

        request.model_state.pose.position.x = float(pathPoint.new_position[0])
        request.model_state.pose.position.y = float(pathPoint.new_position[1])
        request.model_state.pose.position.z = float(pathPoint.new_position[2])
        # pathPoint.new_position[2]=math.atan2(pathPoint.new_velocity[1],pathPoint.new_velocity[0])
        # print("yaw %f",math.atan2(pathPoint.new_velocity[1],pathPoint.new_velocity[0]))
        if self.isStreeing:
            steeringAngle = math.atan2(
                pathPoint.new_velocity[1], pathPoint.new_velocity[0])
            w, x, y, z = transformations.quaternion_from_euler(
            0, 0, float(steeringAngle))
        
        else :
            w, x, y, z = transformations.quaternion_from_euler(
            0, 0, self.targetAngle)  

        request.model_state.pose.orientation.x = x
        request.model_state.pose.orientation.y = y
        request.model_state.pose.orientation.z = z
        request.model_state.pose.orientation.w = w

        request.model_state.twist.linear.x = float(pathPoint.new_velocity[0])
        request.model_state.twist.linear.y = float(pathPoint.new_velocity[1])
        request.model_state.twist.linear.z = float(pathPoint.new_velocity[2])

        request.model_state.twist.angular.x = 0.0
        request.model_state.twist.angular.y = 0.0
        request.model_state.twist.angular.z = 0.0

        future = self.sendTargetStateClient.call_async(request=request)
        # future.add_done_callback(
        #     partial(self.callBack_to_Response_Recieve)
        # )

    def sendNextAngle(self, pathPoint):
        request = SetModelState.Request()

        request.model_state.pose.position.x = float(pathPoint[0])
        request.model_state.pose.position.y = float(pathPoint[1])
        request.model_state.pose.position.z = float(pathPoint[2])
        # pathPoint.new_position[2]=math.atan2(pathPoint.new_velocity[1],pathPoint.new_velocity[0])
        # print("yaw %f",math.atan2(pathPoint.new_velocity[1],pathPoint.new_velocity[0]))
        w, x, y, z = transformations.quaternion_from_euler(0, 0, pathPoint[5])
        request.model_state.pose.orientation.x = x
        request.model_state.pose.orientation.y = y
        request.model_state.pose.orientation.z = z
        request.model_state.pose.orientation.w = w

        request.model_state.twist.linear.x = 0.0
        request.model_state.twist.linear.y = 0.0
        request.model_state.twist.linear.z = 0.0

        request.model_state.twist.angular.x = 0.0
        request.model_state.twist.angular.y = 0.0
        request.model_state.twist.angular.z = float(pathPoint[11])

        future = self.sendTargetStateClient.call_async(request=request)
        # future.add_done_callback(
        #     partial(self.callBack_to_Response_Recieve)
        # )

    def callBack_to_Response_Recieve(self, future):
        try:
            response = future.result()
            self.responseReceived = response.success
        except Exception as e:
            self.get_logger().error("Service call Failed %r" % (e,))
        
    def publishNextTargetState(self):
        i = 0
        first_output, out_list = None, []
        res = Result.Working
        while res == Result.Working:
            res = self.otg.update(self.inp, self.out)

            print('\t'.join([f'{self.out.time:0.3f}'] +
                  [f'{p:0.3f}' for p in self.out.new_position]))
            out_list.append(copy(self.out))

            self.out.pass_to_input(self.inp)

            if not first_output:
                first_output = copy(self.out)
            self.sendNextTarget(self.out)
            time.sleep(0.65)
            print(self.out)

        # print(f'Calculation duration: {first_output.calculation_duration:0.1f} [µs]')
        # print(f'Trajectory duration: {first_output.trajectory.duration:0.4f} [s]')

        # while i<len(self.path):
        #     print(self.path[i])
        #     # if(self.checkError(self.path[i])):
        #     #     i+=1
        #     self.sendNextTarget(self.path[i])
        #     i+=1
        #     time.sleep(6)

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
