import pandas as pd
import matplotlib.pyplot as plt

plt.rcParams["figure.figsize"] = [7.50, 3.50]
plt.rcParams["figure.autolayout"] = True

#headers = ['errorX', 'Time']
columns = ["errorX","errorY","errorZ","estimatedX","estimatedY","estimatedZ","actualX","actualY","actualZ","PercenterrorX","PercenterrorY","PercenterrorZ","timediff"]

df = pd.read_csv('src/visual_odometry/visual_odometry/ORB.csv', usecols=columns)
plt.figure()

plt.plot(df.timediff, df.errorX,"-b", label="Error X")
plt.plot(df.timediff, df.errorY,"-r", label="Error Y")
plt.plot(df.timediff, df.errorZ,"-g", label="Error Z")
plt.xlabel("Time in seconds")
plt.ylabel("Error in metres")
plt.legend(loc="upper left")
plt.title("Measuring error on axes")
plt.figure()
ax = plt.axes(projection ='3d')
ax.plot3D(df.estimatedX, df.estimatedY,df.estimatedZ,"-b", label="Estimated")
ax.plot3D(df.actualX, df.actualY,df.actualZ,"-r", label="Actual")
# plt.plot(df.estimatedX, df.estimatedY,"-b", label="Estimated")
# plt.plot(df.actualX, df.actualY,"-r", label="Actual")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend(loc="upper left")
plt.title("Estimated vs Actual co-ordinates in Visual Odometry")
plt.figure()
plt.plot(df.estimatedX, df.estimatedY,"-b", label="Estimated")
plt.plot(df.actualX, df.actualY,"-r", label="Actual")
plt.xlabel("X")
plt.ylabel("Y")
plt.legend(loc="upper left")
plt.title("Estimated vs Actual co-ordinates (metres) in Visual Odometry")
plt.show()