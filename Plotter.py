import matplotlib.pyplot as plt
import numpy as np
import pygame 
class Plot:
    def __init__(self,dt):
        plt.ion()
        self.dt =dt
        self.Velocity_dt=dt
        self.Omega_dt=dt
        self.Theta_dt=dt
        self.sim_dt=dt
        #plot the traj versus path
        self.Traj_vs_Path,self.ax1 =plt.subplots()
        self.ax1.set_title("Trajtory vs Path")
        self.ax1.set_xlabel("x")
        self.ax1.set_ylabel("y")
        self.x_data=[]
        self.y_robot_data=[]
        self.y_path_data=[]
        self.line1_robot,=self.ax1.plot(self.x_data,self.y_robot_data,label="Robot Path")
        self.line1_path,=self.ax1.plot(self.x_data,self.y_path_data,label=" Refrence Path")
        self.ax1.legend()

        self.Velocity_vs_Time,self.ax2 = plt.subplots()
        self.ax2.set_title("Velocity vs time")
        self.velocit_data =[0]
        self.time_data_v = [0]
        self.line2,=self.ax2.plot(self.time_data_v,self.velocit_data,label="Velocity verses time")
        self.ax2.legend()


        self.Omega_vs_time,self.ax3=plt.subplots()
        self.ax3.set_title("Omega vs time")
        self.omega_data =[0]
        self.time_data_o = [0]
        self.line3,=self.ax3.plot(self.time_data_o,self.omega_data,label="Omega verses time")
        self.ax3.legend()


        self.Theta_vs_time,self.ax4=plt.subplots()
        self.ax4.set_title("Theta Vs Time")
        self.Theta_data =[0]
        self.time_data_theta = [0]
        self.line4,=self.ax4.plot(self.time_data_theta,self.Theta_data,label="theta verses time")
        self.ax4.legend()

        #Logs
        self.Logs,self.ax5=plt.subplots()
        self.ax5.axis("off")
        self.log_text = self.ax5.text(0.1, 0.8, "", fontsize=12)

        self.MaxOvershoot = 0
        self.steady_state_error=float('inf')
        self.settlingTime=float('inf')

    def Plot_Traj_vs_Pah(self,robot_y,path_y,x):
        #increment the pooting time for log calc
        self.sim_dt+=self.sim_dt
        #calculatint the max overshoot the value shall settle on the right one after some time 
        if (self.MaxOvershoot<abs(robot_y - path_y))& (abs(robot_y)>abs(path_y) ):
            self.MaxOvershoot=abs(robot_y - path_y)

      #calculating the settlin thime the number will increase and settle on the right value
        if(abs((robot_y-path_y)/path_y)>0.2):
            self.settlingTime=self.sim_dt
      #calculating the steady state error the error shall decrease until approximitly hold steady on the value 
        self.steady_state_error=abs(robot_y-path_y)
        self.x_data.append(x)
        self.y_path_data.append(path_y)
        self.y_robot_data.append(robot_y)
        self.line1_robot.set_xdata(self.x_data)
        self.line1_robot.set_ydata(self.y_robot_data)
        self.line1_path.set_xdata(self.x_data)
        self.line1_path.set_ydata(self.y_path_data)
        self.ax1.relim()
        self.ax1.autoscale()
        plt.draw()
        plt.pause(0.001)

    def Plot_Velocity_vs_Time (self,velocity):
        self.velocit_data.append(velocity)
        self.Velocity_dt+=self.dt
        self.time_data_v.append(self.Velocity_dt)
        self.line2.set_xdata(self.time_data_v)
        self.line2.set_ydata(self.velocit_data)
        self.ax2.relim()
        self.ax2.autoscale()
        plt.draw()
        plt.pause(0.001)

    def Plot_Omega_vs_Time (self,omega):
        self.omega_data.append(omega)
        self.Omega_dt+=self.dt
        self.time_data_o.append(self.Omega_dt)
        self.line3.set_xdata(self.time_data_o)
        self.line3.set_ydata(self.omega_data)
        self.ax3.relim()
        self.ax3.autoscale()
        plt.draw()
        plt.pause(0.001)
    def Plot_Theta_vs_Time (self,theta):
        self.Theta_data.append(theta)
        self.Theta_dt+=self.dt
        self.time_data_theta.append(self.Theta_dt)
        self.line4.set_xdata(self.time_data_theta)
        self.line4.set_ydata(self.Theta_data)
        self.ax4.relim()
        self.ax4.autoscale()
        plt.draw()
        plt.pause(0.001)
    
    def Print_logs(self) :
        self.log_text.set_text(
        f"Overshoot: {self.MaxOvershoot:.2f}\n"
        f"Settling Time: {self.settlingTime:.2f}\n"
        f"Stead sate error: {self.steady_state_error:.2f}"
    )




class Visualization:

    def __init__(self, dt):

        pygame.init()

        self.screen = pygame.display.set_mode((700, 600))
        self.clock = pygame.time.Clock()

        self.running = True
        self.dt = dt
        self.accum = 0

        self.max_points = 100

        self.robot_points = [None]*self.max_points
        self.Path_points = [None]*self.max_points

        self.front = 0
        self.rear = 0

        self.theta = 0

        self.robot = pygame.image.load("assets/car.PNG")

        w = self.robot.get_width()
        h = self.robot.get_height()

        self.robot = pygame.transform.scale(self.robot,(w//50 , h//50))
        self.rect = self.robot.get_rect()

        # عرض الخط = عرض العربية
        self.line_width = self.robot.get_width()

        # surface للـ shadow
        self.shadow_surface = pygame.Surface(
            self.screen.get_size(),
            pygame.SRCALPHA
        )


    def Get_points(self):

        return np.random.rand()*10,4,45


    def Play(self):

        plot_right = self.screen.get_width()*3/4
        center_y = self.screen.get_height()/2

        while self.running:

            for event in pygame.event.get():

                if event.type == pygame.QUIT:
                    self.running = False


            self.screen.fill("black")
            self.shadow_surface.fill((0,0,0,0))


            # physics
            while self.accum >= self.dt:

                robot_y, path_y, self.theta = self.Get_points()

                self.robot_points[self.rear] = (0,robot_y)
                self.Path_points[self.rear] = (0,path_y)

                self.rect.center = (
                    plot_right,
                    center_y + robot_y
                )

                if self.rear >= self.max_points-1:

                    self.front = (self.front + 1) % self.max_points
                    self.rear = 0

                else:

                    self.rear += 1


                self.accum -= self.dt


            rotated = pygame.transform.rotate(self.robot,self.theta)
            rect = rotated.get_rect(center=self.rect.center)

            robot_draw = []
            path_draw = []

            count = (self.rear-self.front) % self.max_points

            if count == 0:
                count = 1

            step = plot_right / count

            t = 0
            i = (self.rear-1) % self.max_points


            while True:

                if self.robot_points[i] is not None:

                    _, ry = self.robot_points[i]
                    _, py = self.Path_points[i]

                    robot_draw.append(
                        (plot_right - t, center_y + ry)
                    )

                    path_draw.append(
                        (plot_right - t, center_y + py)
                    )

                    t += step


                if i == self.front:
                    break

                i = (i - 1) % self.max_points


            # رسم path shadow
            if len(path_draw) > 1:

                pygame.draw.lines(
                    self.shadow_surface,
                    (0,255,0,80),  # alpha = shadow
                    False,
                    path_draw,
                    self.line_width
                )


            # رسم robot line
            if len(robot_draw) > 1:

                pygame.draw.lines(
                    self.screen,
                    "white",
                    False,
                    robot_draw,
                    self.line_width
                )


            # عرض shadow layer
            self.screen.blit(self.shadow_surface,(0,0))


            # رسم العربية
            self.screen.blit(rotated,rect)


            pygame.display.update()

            frame_dt = self.clock.tick(60)/1000
            self.accum += frame_dt


        pygame.quit()


########################## test ##########################

def main():

    v = Visualization(0.1)
    v.Play()


main()
