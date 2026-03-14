import numpy as np
import time
from vsi import gateway
#Robot class
class Robot :
    def __init__(self,x,y,theta):
        self.__x=x
        self.__y=y
        self.__theta=theta
         #here i will inser some noise and disterbunce in  the reading of y to 
        #simulate the real world noise which is commonly with +0.2 or -0.2
    def get_pos(self):
        return self.__x,self.__y,self.__theta
    def set_pos(self,x,y):
        self.__x=x
        self.__y=y
        
    
    def move(self,v,omega ,dt):
        self.__x=self.__x+v* np.cos(self.__theta)*dt
        self.__y=self.__y+v* np.sin(self.__theta)*dt
        self.__theta =self.__theta+omega*dt

class Path:

    def __init__(self,ln):
         self.path_length=ln
              
    def straight_Path(self,x):
         if(x<self.path_length):
            return 1
         else:
             return None
        
    def Curved_Path(self,x):
         if(x<self.path_length):
             return np.sin(x) 
         else:
             return None
             
            
class Simulator:
    def __init__(self,path_type,dt):
        self.dt = dt
        self.path_type=path_type
        self.robot=Robot(
    np.random.uniform(0,2),
    np.random.uniform(-2,2),
    np.random.rand()*2*np.pi
)
        self.path=Path(300)
                    
    def step(self,v,omega):
        self.robot.move(v,omega,self.dt)
        x,y,theta = self.robot.get_pos()
        if(self.path_type=="straight"):
            y_ref =self.path.straight_Path(x)
        else:
            y_ref=self.path.Curved_Path(x)
        
        
        if y_ref is None:
         y_ref = 0

         x_ref = x
        
        return x_ref,y_ref

    def run(self):

            v = 1
            omega = 0

            while True:

                x_ref, y_ref = self.step(v, omega)
                x,y,theta =self.robot.get_pos()

                print("robot X Coordinate", x,"robot Y Coordinate", y,"Path Y Coordinate","Path X X Coordinate", y_ref,x_ref)   
                time.sleep(self.dt)


def main():
    sim =Simulator("straight",0.01)
    port=gateway.openPort("Simulator")
    while True:
        msg=port.read()
        v=msg["v"]
        omega=msg["omega"]
        x_ref,y_ref=sim.step(v,omega)
    
        
        x_robot, y_robot,theta=sim.robot.get_pos() 

       
        port.write({
            "x_robot": x_robot,
            "y_robot": y_robot,
            "theta": theta,
            "x_path":x_ref,
            "y_path": y_ref
        })
if __name__ == "__main__":

    main()
