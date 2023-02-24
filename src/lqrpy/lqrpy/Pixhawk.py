import threading
from pymavlink import mavutil
import serial
import time
from lqrpy.sensor_data_stream import Getinfo
from lqrpy.commands import Commands


class Pixhawk:
    def __init__(self):
        # try:
        #     self.master = mavutil.mavlink_connection("/dev/ttyACM0", baud=115200)
        # except:
        #     self.master = mavutil.mavlink_connection("/dev/ttyACM1", baud=115200)
        self.defaultSpeed = 100
        self.master = self.init_px4()
        self.heartBeats = threading.Thread(target=self.PixheartBeats)
        self.heartBeats.start()
        self.AUV_control = Commands(self.master)
        self.sensors = Getinfo(self.master)

    def PixheartBeats(self):
        while True:
            # print("Heart Beat")
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                0,
            )
            time.sleep(1)

    def init_px4(self):
        serials = Pixhawk.findSerial()
        if serials is not None:
            master = mavutil.mavlink_connection(serials[0], baud=115200)
        else:
            print("Unable to establish connection")
            return
        master.wait_heartbeat()
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )
        return master

    def findSerial():
        serials = []
        gen_port = "/dev/ttyACM"
        i = 0
        while i < 20:
            if len(serials) == 2:
                return serials
            port = gen_port + "" + str(i)
            i = i + 1
            try:
                ser = serial.Serial(port)

                serials.append(ser.name)

            except serial.SerialException:
                continue

    def arm(self):
        self.AUV_control.arm()

    def disarm(self):
        self.AUV_control.disarm()

    def set_mode(self, mode):
        self.AUV_control.setFlight_mode(mode)

    def forward(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_forward(1500 + speed)

    def backward(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_forward(1500 - speed)

    def left(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_lateral(1500 - speed)

    def right(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_lateral(1500 + speed)

    def down(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_throttle(1500 + speed)

    def up(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_throttle(1500 - speed)

    def yaw_clockwise(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_yaw(1500 + speed)

    def yaw_anticlockwise(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_yaw(1500 - speed)

    def pitch_clockwise(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_pitch(1500 + speed)

    def pitch_anticlockwise(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_pitch(1500 - speed)

    def roll_clockwise(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_roll(1500 + speed)

    def roll_anticlockwise(self, pwm=None):
        speed = self.defaultSpeed
        if pwm is not None:
            speed = self.__map(pwm)
        self.AUV_control.set_roll(1500 - speed)

    def yaw_analysis(self, my_heading, go_to_heading):
        if my_heading == go_to_heading:
            return ""
        if my_heading <= 180:
            a = my_heading
            b = a + 90
            c = b + 90
            if a <= go_to_heading <= b or b <= go_to_heading <= c:
                return "right"
            return "left"
        else:
            a = my_heading
            b = a - 90
            c = b - 90
            if b <= go_to_heading <= a or c <= go_to_heading <= b:
                return "left"
            return "right"

    def rotate90clockwise(self):
        currentHeading = self.getLocalHeading()
        upperDegrees = 0
        if currentHeading > 270:
            upperDegrees = 1

        targetBearing = currentHeading + 90

        correction = 0
        correctBearing = 0
        self.yaw_clockwise(10)
        time.sleep(0.2)
        while True:
            currentHeading = self.getLocalHeading()
            dir = self.yaw_analysis(currentHeading, targetBearing)

            if dir == "right":
                print(targetBearing, currentHeading)

                if correction > 0:
                    if abs(targetBearing - currentHeading) > 45:
                        self.forward(10)
                    else:
                        self.yaw_clockwise(5)
                else:
                    # self.forward(10)
                    self.yaw_clockwise(10)

            else:
                print(targetBearing, currentHeading)
                # self.forward(20)
                self.yaw_anticlockwise(5)
                correction = 1
            print(correctBearing)
            if (
                abs(targetBearing - currentHeading) < 1
                or abs(targetBearing - currentHeading) == 359
            ):
                correctBearing += 1
            if correctBearing > 60:
                self.stop()
                break

    def rotate90anticlockwise(self):
        currentHeading = self.getLocalHeading()

        targetBearing = currentHeading - 90
        if targetBearing <= 0:
            targetBearing = targetBearing + 360

        while True:
            currentHeading = self.getLocalHeading()

            if abs(targetBearing - currentHeading) > 10:
                self.yaw_anticlockwise(10)
            else:
                break

    def setLocalNorth(self):
        self.LocalNorth = self.getHeading()

    def getLocalHeading(self):
        localHeadingDegree = self.getHeading() - self.LocalNorth
        if localHeadingDegree < 0:
            localHeading = localHeadingDegree + 360
        else:
            localHeading = localHeadingDegree
        return localHeading

    def lockOnLocalNorth(self):
        heading = self.getLocalHeading()
        if heading <= 180:
            while heading < 359 and heading > 1:
                heading = self.getLocalHeading()
                self.yaw_anticlockwise(20)
        else:
            while heading < 359 and heading > 1:
                heading = self.getLocalHeading()
                self.yaw_clockwise(20)

    def lockOnLocalEast(self):
        heading = self.getLocalHeading()
        if heading > 90 and heading < 270:
            while heading not in range(89, 91):
                heading = self.getLocalHeading()
                self.yaw_anticlockwise(20)
        else:
            while heading != (89, 91, 0.1):
                heading = self.getLocalHeading()
                self.yaw_clockwise(20)

    def lockOnLocalSouth(self):
        heading = self.getLocalHeading()
        if heading > 180:
            while heading not in range(179, 181):
                heading = self.getLocalHeading()
                self.yaw_anticlockwise(20)
        else:
            while heading != (179, 181, 0.1):
                heading = self.getLocalHeading()
                self.yaw_clockwise(20)

    def lockOnLocalWest(self):
        heading = self.getLocalHeading()
        if heading < 90 or heading > 270:
            while heading not in range(269, 271):
                heading = self.getLocalHeading()
                self.yaw_anticlockwise(20)
        else:
            while heading != (269, 271, 0.1):
                heading = self.getLocalHeading()
                self.yaw_clockwise(20)

    def getHeading(self):
        orientation = self.getOriantation()
        currentHeading = self.convertToDegree(orientation["yaw"])
        return currentHeading

    def convertToDegree(self, radian):
        degrees = int((radian * 180) / 3.14)
        if degrees >= 0:
            return degrees
        else:
            return degrees + 360

    def stop(self):
        self.AUV_control.set_roll(1500)
        self.AUV_control.set_pitch(1500)
        self.AUV_control.set_yaw(1500)
        self.AUV_control.set_throttle(1500)
        self.AUV_control.set_lateral(1500)
        self.AUV_control.set_forward(1500)

    def __map(self, pwm):
        return int(pwm * 400 / 100)

    def setLowSpeed(self):
        self.defaultSpeed = 100

    def setMeduimSpeed(self):
        self.defaultSpeed = 200

    def setHighSpeed(self):
        self.defaultSpeed = 300

    def getOriantation(self):
        return self.sensors.getAttitude().to_dict()

    def getIMU(self):
        return self.sensors.getIMU().to_dict()

    def getNav(self):
        return self.sensors.getNav().to_dict()
