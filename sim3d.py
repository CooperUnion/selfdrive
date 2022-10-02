import numpy as np
import matplotlib.pyplot as plt



dt=0.01 #time
N=1000 #number of
x=np.zeros(N) # 1d array of N elements
y=np.zeros(N) #y
t= np.zeros(N) #time
theta = np.zeros(N)
xdot= np.zeros(N) #derv of x 
ydot= np.zeros(N) #deriv of y
omega = np.zeros(N) #omega 
vmag= np.zeros(N) #velocity along car's x-axis
delta= np.zeros(N) #steering angle in radians
l= 2  #distance between front and back wheel
vmagdot= np.ones(N) 


xdotdot=1 #acceleration
ydotdot=1
vmag[0]=10 #initial velocity

#define initial conditions
x[0]=2
y[0]=0
xdot[0]= vmag[0]*np.cos(theta[0])
ydot[0]= vmag[0]*np.sin(theta[0])


def calculate_purepursuit(x,y,theta):
    return np.clip(0.5*x,-0.5, 0.5) 

for i in range (1,N):
    vmag[i]= vmag[i-1]+vmagdot[i-1]*dt
    t[i]= i*dt
    xdot[i]= vmag[i]*np.cos(theta[i])
    ydot[i]= vmag[i]*np.sin(theta[i])
    x[i]= x[i-1]+xdot[i-1]*dt
    y[i]= y[i-1]+ydot[i-1]*dt
    delta[i]=calculate_purepursuit(x[i],y[i],theta[i])  
    omega[i]= (vmag[i]*np.tan(delta[i]))/l
    theta[i]= theta[i-1]+omega[i-1]*dt

plt.figure(2)
plt.plot(x,y,'b')



plt.figure(0)
plt.plot(t,x,'r')
plt.plot(t, xdot, 'b')


plt.figure(1)
plt.plot(t, y,'y')
plt.plot(t, ydot, 'g')
plt.show()

