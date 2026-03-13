import numpy as np
import time
from Plotter import Plot   # غير الاسم لو الملف اسمه حاجة تانية

dt = 0.05

plotter = Plot(dt)

x = 0
theta = 0

while True:

    # simulate robot motion
    x += dt
    path_y = np.sin(x)
    robot_y = np.sin(x) + np.random.normal(0,0.1)

    velocity = 1
    omega = np.cos(x)

    theta += omega*dt

    # send data to plots
    plotter.Plot_Traj_vs_Pah(robot_y,path_y,x)
    plotter.Plot_Velocity_vs_Time(velocity)
    plotter.Plot_Omega_vs_Time(omega)
    plotter.Plot_Theta_vs_Time(theta)

    plotter.Print_logs()

    time.sleep(dt)
 