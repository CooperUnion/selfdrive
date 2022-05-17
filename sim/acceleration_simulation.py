import numpy as np
import matplotlib.pyplot as plt
from PID_beard import PIDController

N = 10000 # Num timesteps
Ts = 0.001
kp = 1
ki = 0.5
kd = 0.1

controller = PIDController(kp=kp,ki=ki,kd=kd,Ts=Ts, llim=-100, ulim=100, sigma=1)

t = np.linspace(0,N/Ts, N)
vi = 0
vel=np.zeros(N)
vel_des = 0.447*5 #m/s
ctrl = np.zeros(N)

def xdot(x, u, dt):
    return x + u*dt

def step(vd, va, dt):
    # Get controls
    c = controller.PID(vd, va, dt)
    # project to next velocity
    v_next = xdot(va, c, dt)
    # Append data
    


if __name__ == "__main__":
    for i in range(N-1):
        if i == 0:
            ctrl[i] = controller.PID(vel_des, vel[i])
            vel[i+1] = xdot(vel[i],ctrl[i],Ts)
        else:
            # Append data
            ctrl[i] = (controller.PID(vel_des, vel[i]))
            vel[i+1] = (xdot(vel[i],ctrl[i],Ts))
    
    plt.figure()
    plt.plot(t, ctrl, label='ctrl (acceleration)')
    plt.plot(t, vel, label='velocity (m/s)')
    plt.legend()
    plt.title(f"vel_des:{vel_des}, kp:{kp}, ki:{ki}, kd:{kd}")
    plt.show()
    print('abs')