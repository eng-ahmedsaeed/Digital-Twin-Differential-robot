import numpy as np
from vsi import gateway
class PID:
    def __init__(self,Kp,Kd,Ki,dt):
        self.Kp = Kp
        self.Kd =Kd
        self.Ki=Ki
        self.U_prev=0
        self.dt=dt
        self.Integral=0
    def Pid(self,U):
        self.Integral+=U*self.dt
        Derivative=(U-self.U_prev)/self.dt
        Y=self.Kp*U+self.Kd*Derivative+self.Ki*self.Integral
        self.U_prev=U
        return Y
class Controler:
    def __init__(self,Kp,Kd,Ki,dt):
        self.pid=PID(Kp,Kd,Ki,dt)
        self.v=0
        self.omega=0
    def Control (self,y,y_ref):
  
        error=y_ref-y
        self.omega=self.pid.Pid(error)
        #during the research i found that the omega must have a limit called actuator limit commonly its form -3 to 3 rad /sec 
        #to avoid rabid rotations
        self.omega=max(-3 , min(3 , self.omega))
        #here i made the controller adjust also the speed to increase or decrease it within the actuator limit depending on the error
        #which i set to be 1 m/s we need to dec the speed when error is max to give the robot chance to rotate correctly 
        k = 0.5
        v_max = 1
        self.v = v_max - k*abs(error)
        #here these is added in case the error was so huge that it yileds to  a negative velocity  
        self.v = max(0.1 , self.v)
        
        return self.v,self.omega
    

def main():
    cont= Controler(5,10,12,0.01)
    port=gateway.openPort("Controller")
    msg=port.read()
    x_robot=msg[x_robot]
    y_robot=msg[y_robot]
    x_path=msg[x_path]
    y_path=msg[y_path]
    while True:
        v,omega =cont.Control(y_robot,y_path)
        port.write({
            "v":v,
            "omega":omega

        })
        


        
