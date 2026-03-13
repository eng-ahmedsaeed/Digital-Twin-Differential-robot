import numpy as np
import time
#Robot class
class Robot :
    def __init__(self,x,y,theta):
        self.__x=x
        self.__y=y
        self.__theta=theta
         #here i will inser some noise and disterbunce in  the reading of y to 
        #simulate the real world noise which is commonly with +0.2 or -0.2
    def get_pos(self):
        return self.__x*np.random.normal(0,0.2),self.__y*np.random.normal(0,0.2)
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
        x,y = self.robot.get_pos()
        if(self.path_type=="straight"):
            y_ref =self.path.straight_Path(x)
        else:
            y_ref=self.path.Curved_Path(x)
        
        return x,y,y_ref

    def run(self):

            v = 1
            omega = 0

            while True:

                x, y, y_ref = self.step(v, omega)

                print("robot X Coordinate", x,"robot Y Coordinate", y,"Path Y Coordinate", y_ref)   
                time.sleep(self.dt)


if __name__ == "__main__":

    sim = Simulator("curved", 1)

    sim.run()