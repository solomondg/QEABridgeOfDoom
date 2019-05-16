import scipy.stats

X = [
    0.05,
    -0.05,
    0.1,
    -0.1,
    0.15,
    -0.15,
    0.2,
    -0.2,
    0.25,
    -0.25,
    0.3,
    -0.3
]

Y = [
    0.007,
    -0.007,
    0.015,
    -0.015,
    0.021,
    -0.021,
    0.028,
    -0.028,
    0.036,
    -0.036,
    0.042,
    -0.042
]

slope, _intercept, _r, _p, _std_err = scipy.stats.linregress(X,Y)

def getSpeedForPower(power):
    return slope*power

def getPowerForSpeed(speed):
    return speed/slope
