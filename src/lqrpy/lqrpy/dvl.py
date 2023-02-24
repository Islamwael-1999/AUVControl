import threading
from time import sleep
from wldvl import WlDVL
import time


class Dvl:
    def __init__(self):
        self.dvl = WlDVL("/dev/ttyTHS0")
        self.velocity = 0
        self.position = 0
        stream = threading.Thread(target=self.__updateData)
        stream.start()

    def __updateData(self):
        while True:
            try:
                report = self.dvl.read()
            except:
                continue
            
            if report == None:
                continue
            print("report:")
            print(report)
            
            if report["type"] == "velocity":
                self.velocity = report
            elif report["type"] == "position":
                self.position = report
            sleep(0.01)

    def getVelocity(self):
        return self.velocity

    def getPosition(self):
        return self.position


# dvl = Dvl()
# while True :
#     print('get_Position')
#     print(dvl.getPosition())
#     print('Velocity')
#     print(dvl.getVelocity())
 
#     time.sleep(0.0001)
