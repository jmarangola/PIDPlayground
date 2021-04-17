import math
import matplotlib.pyplot as plt
import numpy as np

dt = 0.01

def d2_theta(V, om, kv=1, tao_0=0.2):
    return (1/tao_0) * (kv*V-om)

def plotVerlet(): 
    """Plot verlet evolution of DC motor characteristics based on the 2nd order ODE 
    """
    theta_result, t_scale = [], []
    om_old = 0
    theta_old = 0
    t = 0
    k1v = d2_theta(theta_old, om_old, t) * dt
    k2y = (om_old  + k1v/2)*dt
    k2v = d2_theta(0, om_old + om_old*dt/2)*dt
    om_new = om_old + k2v
    theta_new = theta_old + k2y
    t += dt
    while t < 1:
        theta_oldest = theta_old
        theta_old = theta_new
        om_old = om_new
        theta_new = 2 * theta_old - theta_oldest + d2_theta(1, om_old) * pow(dt, 2)
        theta_result.append(theta_oldest)
        t_scale.append(t-2*dt)
        om_new = (theta_new - theta_oldest) / (2*dt)
        t += dt  
    plt.plot(t_scale, theta_result)
    plt.show()
    return (t_scale, theta_result)
    
if __name__ == "__main__":
    kp = float(input("Kp:"))
    ki = float(input("Ki: "))
    kd = float(input("Kd: "))
    # PID-related
    setpoint = 1
    lastError = 0.0
    integral = 0.0
    
    # verlet evolution
    theta_result, t_scale, V_control = [], [], []
    om_old = 0
    theta_old = 0
    t = 0
    k1v = d2_theta(theta_old, om_old, t) * dt
    k2y = (om_old  + k1v/2)*dt
    k2v = d2_theta(0, om_old + om_old*dt/2)*dt
    om_new = om_old + k2v
    theta_new = theta_old + k2y
    t += dt
    while t < 3:
        theta_oldest = theta_old
        theta_old = theta_new
        om_old = om_new
        
        # PID:
        error = setpoint - theta_old
        integral += error*dt*ki
        proprtional = error*kp
        derivative = kd*(lastError - error)/dt
        V = integral + proprtional + derivative
        lastError = error
        # Verlet evolution
        theta_new = 2 * theta_old - theta_oldest + d2_theta(V, om_old) * pow(dt, 2)
        theta_result.append(theta_oldest)
        t_scale.append(t-2*dt)
        om_new = (theta_new - theta_oldest) / (2*dt)
        t += dt  
        
    plt.plot(t_scale, theta_result)
    x = 0
    plt.show()
    
    
    