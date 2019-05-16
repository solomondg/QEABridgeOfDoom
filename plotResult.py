from doomPathDefs import *
from neatolibs.pathlibs.mathutil.Pose2d import Pose2d
from neatolibs.pathlibs.mathutil.Twist2d import Twist2d
from units import unit
import math
import matplotlib.pyplot as plt
import numpy as np
import scipy as sp
import scipy.interpolate

with open('traj.csv', 'r') as f:
    traj = np.asarray([[float(i) for i in j.split(',')] for j in f.read().split('\n')[:-1]])

with open('robot_pos_log.csv', 'r') as f:
    log = np.asarray([[float(i) for i in j.split(',')] for j in f.read().split('\n')[:-1]])

lV = np.gradient(log[:,1], log[:,0])
rV = np.gradient(log[:,2], log[:,0])

plt.plot(log[:,0], lV, '--')
plt.plot(log[:,0], rV, '--')
plt.plot(traj[:,0], traj[:,-2])
plt.plot(traj[:,0], traj[:,-1])
plt.ylabel("Velocity (m/s)")
plt.xlabel("Time (s)")
plt.legend(["Left Actual", "Right Actual", "Left profile ", "Right profile "], loc=1)
plt.title("Profile versus actual wheel velocities")

plt.show()

simRobot_pose = Pose2d(
    getPoint(0),
    getTangent(0)
)

simx = []
simy = []
simt = []
simw = []
simv = []
for i in range(len(log[:,0])-1):
    simt.append(log[i,0])
    dl = log[i+1,1]-log[i,1]
    dr = log[i+1,2]-log[i,2]
    w = (dr-dl)/0.247613
    v = (dl+dr)/2
    simw.append(w)
    simv.append(v)

    simRobot_pose = simRobot_pose.relativeTransformBy(
        Pose2d.exp(Twist2d(dx=v,dy=0,dtheta=w))
    )

    #simRobot_pose.rotation = simRobot_pose.rotation.rotateBy(Rotation2d.createFromRadians(w))
    #simRobot_pose.translation = Translation2d(v,0).rotateByOrigin(simRobot_pose.rotation)

    simx.append(simRobot_pose.translation.x)
    simy.append(simRobot_pose.translation.y)

trajx = []
trajy = []
trajw = []
trajv = []
trajt = []
simRobot_pose = Pose2d(
    getPoint(0),
    getTangent(0)
)

for i in range(len(traj[:,0])-1):
    trajt.append(traj[i,0])
    dt = traj[i+1,0]-traj[i,0]
    dl = traj[i,-2]*dt
    dr = traj[i,-1]*dt
    w = (dr-dl)/0.247613
    v = (dl+dr)/2
    trajv.append(v*7)
    trajw.append(w*7)
    simRobot_pose = simRobot_pose.relativeTransformBy(
        Pose2d.exp(Twist2d(dx=v,dy=0,dtheta=w))
    )
    trajx.append(simRobot_pose.translation.x)
    trajy.append(simRobot_pose.translation.y)
plt.plot(trajx, trajy)
plt.plot(simx, simy, '--')
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.legend(["Bridge path", "Traveled path"],loc=1)
plt.title("Theoretical path vs. path traveled by robot")
plt.show()

plt.plot(trajt,trajv)
plt.plot(trajt,trajw)
plt.plot(simt,simv,'--')
plt.plot(simt,simw,'--')
plt.xlabel("Time (s)")
plt.ylabel("Linear vel (m/s), Angular vel (rad/s)")
plt.legend(["Profile linear velocity", "Profile angular velocity", "Robot linear velocity", "Robot angular velocity"])
plt.title("Profile w+v versus robot w+v")
plt.show()

# t, v, w, x, y, theta, lV, rV
xInterp = sp.interpolate.interp1d(traj[:,0],traj[:,4])
yInterp = sp.interpolate.interp1d(traj[:,0],traj[:,5])


# Calculate total error
# I'm interpolating the reference trajectory so it's painless to find ref
# timesteps that match with my robot position point timesteps
acc = 0
for i in range(len(log[:,0])-1):
    t = log[i,0]
    x = simx[i]
    y = simy[i]
    dx = float(xInterp(t)-x)
    dy = float(yInterp(t)-y)
    n = np.linalg.norm(np.asarray([dx,dy]))
    acc += n

acc /= getLength()*(len(log[:,0])-1)
print("Avergage error (m)", acc)
