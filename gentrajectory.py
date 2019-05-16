from doomPathDefs import *
from neatolibs.pathlibs.mathutil.Pose2d import Pose2d
from neatolibs.pathlibs.mathutil.Twist2d import Twist2d
from units import unit
import math
import matplotlib.pyplot as plt

from tqdm import tqdm

neatoMaxSpeed = 0.299999 * unit.m / unit.s
neatoMaxAccel = 0.3 * unit.m / (unit.s ** 2)

profileMaxCruiseVel = neatoMaxSpeed * 0.75
profileMaxAccel = neatoMaxAccel * 0.75

pathLength = getLength() * unit.m
#pathLength = 3.253489154999915 * unit.m

cruiseVelStartTime = profileMaxCruiseVel / profileMaxAccel
dt = 0.02 * unit.s
tVec = [0.0 * unit.s]  # timesteps
xVec = [0.0 * unit.m]  # distance traveled
vVec = [0.0 * unit.m / unit.s]  # velocity
aVec = [0.0 * unit.m / unit.s ** 2]  # acceleration

cruiseVelTime = pathLength / profileMaxCruiseVel - cruiseVelStartTime

if profileMaxCruiseVel * cruiseVelStartTime > pathLength:
    cruiseVelStartTime = math.sqrt((pathLength / profileMaxAccel).to(unit.s**2).m) * unit.s
    cruiseVelEndTime = cruiseVelStartTime
    timeTotal = 2.0 * cruiseVelStartTime
    profile_max_v = profileMaxAccel * cruiseVelStartTime
else:
    cruiseVelEndTime = cruiseVelStartTime + cruiseVelTime
    timeTotal = cruiseVelEndTime + cruiseVelStartTime

while tVec[-1] < timeTotal:
    t = tVec[-1] + dt
    tVec.append(t)

    if t < cruiseVelStartTime:
        aVec.append(profileMaxAccel)
        vVec.append(profileMaxAccel * t)
    elif t < cruiseVelEndTime:
        aVec.append(0.0 * unit.m / unit.s ** 2)
        vVec.append(profileMaxCruiseVel)
    elif t < timeTotal:
        decelStartTime = t - cruiseVelEndTime
        aVec.append(-profileMaxAccel)
        vVec.append(profileMaxCruiseVel - profileMaxAccel * decelStartTime)
    else:
        aVec.append(0.0 * unit.m / unit.s ** 2)
        vVec.append(0.0 * unit.m / unit.s)
    # print(xVec[-1], vVec[-1], dt, vVec[-1]*dt)
    xVec.append(xVec[-1] + vVec[-1] * dt)

plt.plot([t.to(unit.s).m for t in tVec], [x.to(unit.m).m for x in xVec])
plt.plot([t.to(unit.s).m for t in tVec], [v.to(unit.m / unit.s).m for v in vVec])
plt.plot([t.to(unit.s).m for t in tVec], [a.to(unit.m / unit.s ** 2).m for a in aVec])
plt.xlabel("Time (s)")
plt.ylabel("Position (m), Velocity (m/s), Acceleration (m/s^2)")
plt.legend(["Position (m)", "Velocity (m/s)", "Acceleration (m/s^2)"])
plt.show()

v = vVec  # linear velocity
# w = That x Nhat
headings = []
w = []
for i in tqdm(range(0, len(xVec))):
    # get current x
    # get next x
    # get heading at current x
    # get heading at next x
    # delta
    percent = xVec[i].to(unit.m).m / pathLength.to(unit.m).m
    #w_ = getW(percent) * 0.235
    w_ = getW(percent) * vVec[i].to(unit.m/unit.s).m
    w.append(w_)
    headings.append(getTangent(percent).theta)
v = np.asarray([j.to(unit.m / unit.s).m for j in vVec])
w = np.asarray(w)
t = np.asarray([t.to(unit.s).m for t in tVec])

# w_fake = []
# for i in tqdm(range(len(tVec)-1)):
#    pt_0 = (i+0.0)/(len(tVec)-1)
#    pt_1 = (i+1.0)/(len(tVec)-1)
#    th_0 = getTangent(pt_0)
#    th_1 = getTangent(pt_1)
#    w_tmp = th_0.rotateBy(th_1.inverse).theta# * dt.to(unit.s).m
#    w_fake.append(w_tmp)

plt.plot(t, w)
# plt.plot([t.to(unit.s).m for t in tVec[:-1]], w_fake)
plt.show()

wheelbase = 0.247613

traj_l = v - w * (wheelbase / 2)
traj_r = v + w * (wheelbase / 2)

x_ = [getPoint(0).x]
y_ = [getPoint(0).y]
t_ = [getTangent(0).theta]
#for i in tqdm(range(len(traj_l))):
#    v_ = (traj_l[i] + traj_r[i]) / 2
#    w_ = (traj_r[i] - traj_l[i]) / wheelbase
#    t_.append(t_[-1] + w)
#    x_.append(x_[-1] + v_ * np.cos(t_[-1]))
#    y_.append(y_[-1] + v_ * np.sin(t_[-1]))

plt.plot(t, traj_l)
plt.plot(t, traj_r)
plt.show()

t = np.linspace(0, 1, num=500)
X = []
Y = []
for i in t:
    pt = getPoint(i)
    X.append(pt.x)
    Y.append(pt.y)

plt.plot(X, Y)
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
# plt.xlim([-1, 1])
# plt.ylim([-1, 1])
# plt.plot(x_, y_)

simRobot_pose = Pose2d(
    getPoint(0),
    getTangent(0)
)

simx = []
simy = []
simt = []

dt = dt.to(unit.s).m

for i in range(len(traj_r)):
    a = float((traj_r[i] - traj_l[i]) / wheelbase)
    b = float((traj_l[i] + traj_r[i]) / 2)
    simRobot_pose = simRobot_pose.relativeTransformBy(
        Pose2d.exp(Twist2d(dx=b * dt, dy=0.0, dtheta=a * dt))
    )
    simx.append(simRobot_pose.translation.x)
    simy.append(simRobot_pose.translation.y)
    simt.append(simRobot_pose.rotation.theta)

plt.plot(simx, simy)
plt.show()
plt.plot([i.m for i in tVec], headings)
plt.plot([i.m for i in tVec], simt)
plt.show()

csv = ""

print(simx[-1], simy[-1])
print(getPoint(1.0).x, getPoint(1.0).y)

# Csv format:
# t, v, w, x, y, theta, lV, rV
for i in range(len(tVec)):
    csv += "{0},{1},{2},{3},{4},{5},{6},{7}\n".format(
        tVec[i].to(unit.s).m,                                               # t
        v[i],                                                               # v
        w[i],                                                               # w
        getPoint(xVec[i].to(unit.m).m/pathLength.to(unit.m).m).x,           # x
        getPoint(xVec[i].to(unit.m).m/pathLength.to(unit.m).m).y,           # y
        getTangent(xVec[i].to(unit.m).m/pathLength.to(unit.m).m).radians,   # theta
        traj_l[i],                                                          # lV
        traj_r[i]                                                           # rV
    )


with open("traj.csv", "w") as f:
    f.write(csv)
