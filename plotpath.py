from doomPathDefs import *
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

t = np.linspace(0, 1, num=500)
X = []
Y = []
for i in tqdm(t):
    pt = getPoint(i)
    X.append(pt.x)
    Y.append(pt.y)

plt.plot(X,Y)
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.xlim([-1, 1])
plt.ylim([-1, 1])

vecpts = [0.1, 0.55, 0.7]
vecpts = np.linspace(0,1,num=10)

for i in vecpts:
    pt = getPoint(i)
    pt_tan = getTangent(i)
    pt_norm = getNormal(i)
    plt.quiver(
        [pt.x, pt.x],
        [pt.y, pt.y],
        [pt_tan.cos, pt_norm.cos],
        [pt_tan.sin, pt_norm.sin], color=['red','blue']
    )

plt.title("S-curve bridge, with tangent and normal vectors")
plt.show()
