import numpy as np
import matplotlib.pyplot as plt

dt=0.01 #time
N=1000 #number of
x=np.zeros(N) #1d array of N elements
t= np.zeros(N)
xdot= np.zeros(N)
xdotdot=1 #acceleration

#define initial conditions
x[0]=0
xdot[0]=0


for i in range (1,N):
    t[i]= i*dt
    xdot[i]= xdot[i-1] + xdotdot*dt
    x[i]= x[i-1]+xdot[i-1]*dt


plt.figure(0)
plt.plot(t,x,'r')
plt.plot(t, xdot, 'b')
plt.show()




