#square brakets are lists
import numpy as np
import matplotlib.pyplot as plt

v = []
dt = 0.01 #time between iterations (sample period)
a = 1 #acceleration constant for now
vi = 0 # initial velocity
kp = 10
vd = 5


for i in range (0, 1000):
    if (i==0):
        v = [vi]
        t = [0]
        pass
    else:
        ak = (vd - v[i-1])*kp
        vnext = v[i-1] + ak*dt #first order integration over sample period
        t.append (i*dt)
        v.append (vnext)

plt.figure()
plt.plot(t,v)
plt.show()
print(v)


