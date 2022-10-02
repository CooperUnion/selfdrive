import numpy as np
import matplotlib.pyplot as plt

dt=0.01 #time
N=1000 #number of
x=np.zeros(N) # 1d array of N elements
y=np.zeros(N)
t= np.zeros(N)
xdot= np.zeros(N)
ydot= np.zeros(N)
xdotdot=1 #acceleration
ydotdot=12

#define initial conditions
x[0]=0
y[0]=0
xdot[0]=0
ydot[0]=0


for i in range (1,N):
    t[i]= i*dt
    xdot[i]= xdot[i-1] + xdotdot*dt
    ydot[i]= ydot[i-1] + ydotdot*dt
    x[i]= x[i-1]+xdot[i-1]*dt
    y[i]= y[i-1]+ydot[i-1]*dt


plt.figure(0)
plt.plot(t,x,'r')
plt.plot(t, xdot, 'b')
plt.show()

plt.figure(1)
plt.plot(t, y,'y')
plt.plot(t, ydot, 'g')
plt.show()


