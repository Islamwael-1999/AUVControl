import matplotlib.pyplot as plt
from klampt.model.trajectory import HermiteTrajectory
from math import pi
class TrajectoryPlanner(HermiteTrajectory):
    def __init__(self,times,positions,velocities):
        super().__init__(times=times,milestones=positions,dmilestones=velocities)
    def getPosition(self,timestamp):
        return self.eval_state(timestamp)[0:5]
    def getVelocity(self,timestamp):
        return self.eval_state(timestamp)[6:11]
    def getAcceleration(self,timestamp):
        return self.eval_accel(timestamp)
        


#major timestamps
times = [0.0,90.0,160.0,200.0,250.0,300.0]
        #major milestones positions
state = [[0,0,-1,0,0,0],[3,3,-2,0,0,pi/4],[5,5,-2,0,0,pi/2],[3,7,-2,0,0,3*pi/4],[-3,3,-2,0,0,5*pi/4],[0,0,-2,0,0,2*pi]]
        #major milestones velocities
velc= [[0,0,0,0,0,0],[0.1,0.1,0,0,0,0],[0,0.1,0,0,0,0],[-0.1,-0.1,0,0,0,0],[0.1,-0.1,0,0,0,0],[0,0,0,0,0,0]]

traj=TrajectoryPlanner(times=times,positions=state,velocities=velc)

nextState=traj.getVelocity(13)
print(nextState)
timestamps=[]
positionsX=[]
positionsY=[]
VelocitiesX=[]
VelocitiesY=[]
for i in range(0,50*300,1):
    timestamps.append(i/100)
    positionsX.append(traj.getPosition(i/50)[0])
    positionsY.append(-traj.getPosition(i/50)[1])
    VelocitiesX.append(traj.getVelocity(i/50)[0])
    VelocitiesY.append(-traj.getVelocity(i/50)[1])
plt.plot(positionsY,positionsX)
plt.figure()
plt.plot(VelocitiesX,positionsX)
plt.xlabel('y')
plt.ylabel('x')
plt.show()


