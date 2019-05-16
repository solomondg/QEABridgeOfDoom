import math

import numpy as np
#from numpy import cos,sin
from neatolibs.pathlibs.mathutil.Rotation2d import Rotation2d
from neatolibs.pathlibs.mathutil.Translation2d import Translation2d
from neatolibs.pathlibs.mathutil.Types import Vector2d
import sympy
from sympy import symbols, N, integrate, Function, Matrix, cos, sin


u_ = symbols('u', real=True)
eq = Function('t')
eqn = Matrix([
    0.3960 * cos(2.65*(u_+1.4)),
    -0.99 * sin(u_ + 1.4)
    ])

#eqn = Matrix([
#    u_,
#    u_**2
#])

tOffset = 0.0
tMax = 2.9-tOffset

def getPoint(t):
    u = (t+tOffset)*tMax
    eq_ = eqn.subs(u_, u)
    return Translation2d(eq_[0], eq_[1])


def getTangent(t):
    u = (t+tOffset)*tMax
    tan_ = (eqn.diff(u_) / eqn.diff(u_).norm()).subs(u_, u)
    return Rotation2d(float(tan_[0]), float(tan_[1]))

def getNormal(t):
    u = (t+tOffset)*tMax
    tan_ = (eqn.diff(u_) / eqn.diff(u_).norm())
    norm_ = (tan_.diff(u_) / tan_.diff(u_).norm()).subs(u_, u)
    return Rotation2d(float(norm_[0]), float(norm_[1]))

def getLength():
    u_min = tOffset
    u_max = tMax
    return N(integrate(eqn.diff(u_).norm(), (u_, u_min, u_max)))


def getW(t):
    u = (t+tOffset)*tMax
    tan_ = (eqn.diff(u_) / eqn.diff(u_).norm())
    dtan_ = tan_.diff(u_)
    tan_sub = tan_.subs(u_, u)
    dtan_sub = dtan_.subs(u_, u)
    tan_vec = np.asarray([float(tan_sub[0]), float(tan_sub[1])])
    dtan_vec = np.asarray([float(dtan_sub[0]), float(dtan_sub[1])])
    w = np.cross(tan_vec, dtan_vec)
    d = 0.01
    #w = getTangent(u+d/2).rotateBy(Rotation2d.createFromRadians(-getTangent(u-d/2).theta)).theta / d
    return w
