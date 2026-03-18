#!/usr/bin/env python3
from __future__ import print_function
import struct
import sys
import argparse
import math

PythonGateways = 'pythonGateways/'
sys.path.append(PythonGateways)

import VsiCommonPythonApi as vsiCommonPythonApi
import VsiTcpUdpPythonGateway as vsiEthernetPythonGateway


class MySignals:
	def __init__(self):
		# Inputs
		self.x_robot = 0
		self.y_robot = 0
		self.theta = 0
		self.x_path = 0
		self.y_path = 0
		self.x_path_curr = 0
		self.y_path_curr = 0
		self.v = 0
		self.omega = 0



srcMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]
SimulatorMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBA]
ControllerMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBB]
srcIpAddress = [192, 168, 1, 4]
SimulatorIpAddress = [192, 168, 1, 2]
ControllerIpAddress = [192, 168, 1, 3]

SimulatorSocketPortNumber0 = 8071
ControllerSocketPortNumber1 = 8072

Visualizer0 = 0
Visualizer1 = 1


# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions
import matplotlib.pyplot as plt
import numpy as np
import pygame 


class Plot:
    def __init__(self):
        plt.ion()
        

        self.Velocity_dt=0
        self.Omega_dt=0
        self.Theta_dt=0
        self.sim_dt=0
        self.lat_dt=0
        
        # Plot the trajectory vs path
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

        # Combined figure for Omega, Theta, and Velocity (horizontally stacked)
        self.ThetaOmegaVel_figure, (self.ax3, self.ax4, self.ax2_new) = plt.subplots(1, 3, figsize=(15, 5))
        
        # Omega vs time
        self.ax3.set_title("Omega vs time")
        self.omega_data = [0]
        self.time_data_o = [0]
        self.line3, = self.ax3.plot(self.time_data_o, self.omega_data, label="Omega verses time")
        self.ax3.set_xlabel("time")
        self.ax3.set_ylabel("omega")
        self.ax3.legend()

        # Theta vs Time
        self.ax4.set_title("Theta Vs Time")
        self.Theta_data = [0]
        self.time_data_theta = [0]
        self.line4, = self.ax4.plot(self.time_data_theta, self.Theta_data, label="theta verses time")
        self.ax4.set_xlabel("time")
        self.ax4.set_ylabel("theta")
        self.ax4.legend()

        # Velocity vs Time (in combined figure)
        self.ax2_new.set_title("Velocity vs time")
        self.velocit_data_new = [0]
        self.time_data_v_new = [0]
        self.line2_new, = self.ax2_new.plot(self.time_data_v_new, self.velocit_data_new, label="Velocity verses time")
        self.ax2_new.set_xlabel("time")
        self.ax2_new.set_ylabel("velocity")
        self.ax2_new.legend()

        # Combined figure for Logs and Lateral Error (horizontally stacked)
        self.LogsLatError_figure, (self.ax5, self.ax6) = plt.subplots(1, 2, figsize=(12, 5))
        
        # Logs
        self.ax5.axis("off")
        self.ax5.set_title("Performance Logs")
        self.log_text = self.ax5.text(0.1, 0.8, "", fontsize=12)

        # Lateral Error
        self.ax6.set_title("Lateral Error")
        self.ax6.set_xlabel("time")
        self.ax6.set_ylabel("lateral error")
        self.time_data_lat = [0]
        self.lateral_error = [0]
        self.line6, = self.ax6.plot(self.time_data_lat, self.lateral_error, label="Lateral Error")
        self.ax6.legend()

        self.MaxOvershoot = 0
        self.steady_state_error=float('inf')
        self.settlingTime=float('inf')

    def Plot_Traj_vs_Path(self,robot_y,path_y,x,x_path,dt):
        #increment the pooting time for log calc
        self.sim_dt+=dt
        #calculatint the max overshoot the value shall settle on the right one after some time 
        if (self.MaxOvershoot<abs(robot_y - path_y))and (abs(robot_y)>abs(path_y) ):
            self.MaxOvershoot=abs(robot_y - path_y)

      #calculating the settlin thime the number will increase and settle on the right value
        threshold = 0.02  # 2% 
        error = abs(robot_y - path_y)

        if error > threshold:
            self.settlingTime = self.sim_dt
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

    def Plot_Velocity_vs_Time (self,velocity,dt):
        self.velocit_data_new.append(velocity)
        self.Velocity_dt+=dt
        self.time_data_v_new.append(self.Velocity_dt)
        self.line2_new.set_xdata(self.time_data_v_new)
        self.line2_new.set_ydata(self.velocit_data_new)
        self.ax2_new.relim()
        self.ax2_new.autoscale()
        self.ThetaOmegaVel_figure.canvas.draw()
        plt.pause(0.001)
    def Plot_lateral_error_vs_time (self,robot_y,path_y,dt):
        self.lateral_error.append(path_y-robot_y)
        self.lat_dt+=dt
        self.time_data_lat.append(self.lat_dt)
        self.line6.set_xdata(self.time_data_lat)
        self.line6.set_ydata(self.lateral_error)
        self.ax6.relim()
        self.ax6.autoscale()
        self.LogsLatError_figure.canvas.draw()
        plt.pause(0.001)
    def Plot_Omega_vs_Time (self,omega,dt):
        self.omega_data.append(omega)
        self.Omega_dt+=dt
        self.time_data_o.append(self.Omega_dt)
        self.line3.set_xdata(self.time_data_o)
        self.line3.set_ydata(self.omega_data)
        self.ax3.relim()
        self.ax3.autoscale()
        self.ThetaOmegaVel_figure.canvas.draw()
        plt.pause(0.001)
    def Plot_Theta_vs_Time (self,theta,dt):
        self.Theta_data.append(theta)
        self.Theta_dt+=dt
        self.time_data_theta.append(self.Theta_dt)
        self.line4.set_xdata(self.time_data_theta)
        self.line4.set_ydata(self.Theta_data)
        self.ax4.relim()
        self.ax4.autoscale()
        self.ThetaOmegaVel_figure.canvas.draw()
        plt.pause(0.001)
    
    def Print_logs(self) :
        self.log_text.set_text(
        f"Overshoot: {self.MaxOvershoot:.5f}\n"
        f"Settling Time: {self.settlingTime:.5f}\n"
        f"Stead sate error: {self.steady_state_error:.5f}"
    )
        self.LogsLatError_figure.canvas.draw()
        plt.pause(0.001)




import os
class Visualization:

    def __init__(self, dt):

        pygame.init()

        self.screen = pygame.display.set_mode((700, 600))
        self.clock = pygame.time.Clock()

        self.running = True
        self.dt = dt

        self.theta = 0
        self.robot_y = 0
        base_path = os.path.dirname(__file__)
        img_path = os.path.join(base_path, "assets", "car.png")
        self.robot = pygame.image.load(img_path)
        w = self.robot.get_width()
        h = self.robot.get_height()

        self.robot = pygame.transform.scale(self.robot, (w//5, h//5))
        self.center_x = self.screen.get_width() / 2
        self.center_y = self.screen.get_height() / 2

    def Set_points(self, robot_x, robot_y, path_x, path_y, theta):
        """Set the robot position and rotation angle"""
        self.robot_y = robot_y
        self.theta = theta

    def Update_frame(self, dt=None):
        """
        Update one frame - just rotate and move car up/down
        Returns: True if running, False if quit requested
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

        if not self.running:
            pygame.quit()
            return False

        # Clear screen
        self.screen.fill("white")

        # Rotate car based on theta (convert to degrees)
        theta_degrees = self.theta * 180 / 3.14159  # Convert radians to degrees
        rotated = pygame.transform.rotate(self.robot, theta_degrees)
        rect = rotated.get_rect(center=(self.center_x, self.center_y + self.robot_y * 50))

        # Draw car
        self.screen.blit(rotated, rect)

        pygame.display.update()
        self.clock.tick(60)
        
        return True


# End of user custom code region. Please don't edit beyond this point.
class Visualizer:

	def __init__(self, args):
		self.componentId = 2
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50103
        
		self.simulationStep = 0
		self.stopRequested = False
		self.totalSimulationTime = 0
        
		self.receivedNumberOfBytes = 0
		self.receivedPayload = []

		self.numberOfPorts = 2
		self.clientPortNum = [0] * self.numberOfPorts
		self.receivedDestPortNumber = 0
		self.receivedSrcPortNumber = 0
		self.expectedNumberOfBytes = 0
		self.mySignals = MySignals()

		# Start of user custom code region. Please apply edits only within these regions:  Constructor
		self.plot = Plot()
		self.visualization = Visualization(dt=0.01)  # Initialize visualization with 0.01s timestep
		# End of user custom code region. Please don't edit beyond this point.



	def mainThread(self):
		dSession = vsiCommonPythonApi.connectToServer(self.localHost, self.domain, self.portNum, self.componentId)
		vsiEthernetPythonGateway.initialize(dSession, self.componentId, bytes(srcMacAddress), bytes(srcIpAddress))
		try:
			vsiCommonPythonApi.waitForReset()

			# Start of user custom code region. Please apply edits only within these regions:  After Reset

			# End of user custom code region. Please don't edit beyond this point.
			self.updateInternalVariables()

			if(vsiCommonPythonApi.isStopRequested()):
				raise Exception("stopRequested")
			self.establishTcpUdpConnection()
			nextExpectedTime = vsiCommonPythonApi.getSimulationTimeInNs()
			while(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):

				# Start of user custom code region. Please apply edits only within these regions:  Inside the while loop
				#self.updateInternalVariables()
				dt = self.simulationStep * 1e-9
				
				# Update visualization with current signals
				self.visualization.Set_points(
					self.mySignals.x_robot,
					self.mySignals.y_robot,
					self.mySignals.x_path_curr,
					self.mySignals.y_path_curr,
					self.mySignals.theta
				)
				
				# Update visualization frame
				if not self.visualization.Update_frame(dt):
					raise Exception("Visualization closed")
				
				# Update plots
				self.plot.Plot_Traj_vs_Path(self.mySignals.y_robot,self.mySignals.y_path_curr,self.mySignals.x_robot,self.mySignals.x_path_curr,dt)
				self.plot.Plot_Velocity_vs_Time(self.mySignals.v,dt)
				self.plot.Plot_Omega_vs_Time(self.mySignals.omega,dt)
				self.plot.Plot_Theta_vs_Time(self.mySignals.theta,dt)
				self.plot.Plot_lateral_error_vs_time(self.mySignals.y_robot,self.mySignals.y_path_curr,dt)
				self.plot.Print_logs()
				# End of user custom code region. Please don't edit beyond this point.

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")

				if(vsiEthernetPythonGateway.isTerminationOnGoing()):
					print("Termination is on going")
					break

				if(vsiEthernetPythonGateway.isTerminated()):
					print("Application terminated")
					break

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(SimulatorSocketPortNumber0)
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(ControllerSocketPortNumber1)
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=Visualizer+=")
				print("  VSI time:", end = " ")
				print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
				print("ns")
				print("  Inputs:")
				print("\tx_robot =", end = " ")
				print(self.mySignals.x_robot)
				print("\ty_robot =", end = " ")
				print(self.mySignals.y_robot)
				print("\ttheta =", end = " ")
				print(self.mySignals.theta)
				print("\tx_path =", end = " ")
				print(self.mySignals.x_path)
				print("\ty_path =", end = " ")
				print(self.mySignals.y_path)
				print("\tx_path_curr =", end = " ")
				print(self.mySignals.x_path_curr)
				print("\ty_path_curr =", end = " ")
				print(self.mySignals.y_path_curr)
				print("\tv =", end = " ")
				print(self.mySignals.v)
				print("\tomega =", end = " ")
				print(self.mySignals.omega)
				print("\n\n")

				self.updateInternalVariables()

				if(vsiCommonPythonApi.isStopRequested()):
					raise Exception("stopRequested")
				nextExpectedTime += self.simulationStep

				if(vsiCommonPythonApi.getSimulationTimeInNs() >= nextExpectedTime):
					continue

				if(nextExpectedTime > self.totalSimulationTime):
					remainingTime = self.totalSimulationTime - vsiCommonPythonApi.getSimulationTimeInNs()
					vsiCommonPythonApi.advanceSimulation(remainingTime)
					break

				vsiCommonPythonApi.advanceSimulation(nextExpectedTime - vsiCommonPythonApi.getSimulationTimeInNs())

			# End of while loop - graceful shutdown
			if(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):
				vsiEthernetPythonGateway.terminate()
		except Exception as e:
			if str(e) == "stopRequested":
				print("Terminate signal has been received from one of the VSI clients")
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print(f"An error occurred: {str(e)}")
		except:
			vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)



	def establishTcpUdpConnection(self):
		if(self.clientPortNum[Visualizer0] == 0):
			self.clientPortNum[Visualizer0] = vsiEthernetPythonGateway.tcpConnect(bytes(SimulatorIpAddress), SimulatorSocketPortNumber0)

		if(self.clientPortNum[Visualizer1] == 0):
			self.clientPortNum[Visualizer1] = vsiEthernetPythonGateway.tcpConnect(bytes(ControllerIpAddress), ControllerSocketPortNumber1)

		if(self.clientPortNum[Visualizer1] == 0):
			print("Error: Failed to connect to port: Simulator on TCP port: ") 
			print(SimulatorSocketPortNumber0)
			exit()

		if(self.clientPortNum[Visualizer1] == 0):
			print("Error: Failed to connect to port: Controller on TCP port: ") 
			print(ControllerSocketPortNumber1)
			exit()



	def decapsulateReceivedData(self, receivedData):
		self.receivedDestPortNumber = receivedData[0]
		self.receivedSrcPortNumber = receivedData[1]
		self.receivedNumberOfBytes = receivedData[3]
		self.receivedPayload = [0] * (self.receivedNumberOfBytes)

		for i in range(self.receivedNumberOfBytes):
			self.receivedPayload[i] = receivedData[2][i]

		if(self.receivedSrcPortNumber == SimulatorSocketPortNumber0):
			print("Received packet from Simulator")
			receivedPayload = bytes(self.receivedPayload)
			self.mySignals.x_robot, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.y_robot, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.theta, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.x_path, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.y_path, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.x_path_curr, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.y_path_curr, receivedPayload = self.unpackBytes('d', receivedPayload)


		if(self.receivedSrcPortNumber == ControllerSocketPortNumber1):
			print("Received packet from Controller")
			receivedPayload = bytes(self.receivedPayload)
			self.mySignals.v, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.omega, receivedPayload = self.unpackBytes('d', receivedPayload)


		# Start of user custom code region. Please apply edits only within these regions:  Protocol's callback function

		# End of user custom code region. Please don't edit beyond this point.



	def packBytes(self, signalType, signal):
		if isinstance(signal, list):
			if signalType == 's':
				packedData = b''
				for str in signal:
					str += '\0'
					str = str.encode('utf-8')
					packedData += struct.pack(f'={len(str)}s', str)
				return packedData
			else:
				return struct.pack(f'={len(signal)}{signalType}', *signal)
		else:
			if signalType == 's':
				signal += '\0'
				signal = signal.encode('utf-8')
				return struct.pack(f'={len(signal)}s', signal)
			else:
				return struct.pack(f'={signalType}', signal)



	def unpackBytes(self, signalType, packedBytes, signal = ""):
		if isinstance(signal, list):
			if signalType == 's':
				unpackedStrings = [''] * len(signal)
				for i in range(len(signal)):
					nullCharacterIndex = packedBytes.find(b'\0')
					if nullCharacterIndex == -1:
						break
					unpackedString = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
					unpackedStrings[i] = unpackedString
					packedBytes = packedBytes[nullCharacterIndex + 1:]
				return unpackedStrings, packedBytes
			else:
				unpackedVariable = struct.unpack(f'={len(signal)}{signalType}', packedBytes[:len(signal)*struct.calcsize(f'={signalType}')])
				packedBytes = packedBytes[len(unpackedVariable)*struct.calcsize(f'={signalType}'):]
				return list(unpackedVariable), packedBytes
		elif signalType == 's':
			nullCharacterIndex = packedBytes.find(b'\0')
			unpackedVariable = struct.unpack(f'={nullCharacterIndex}s', packedBytes[:nullCharacterIndex])[0].decode('utf-8')
			packedBytes = packedBytes[nullCharacterIndex + 1:]
			return unpackedVariable, packedBytes
		else:
			numBytes = 0
			if signalType in ['?', 'b', 'B']:
				numBytes = 1
			elif signalType in ['h', 'H']:
				numBytes = 2
			elif signalType in ['f', 'i', 'I', 'L', 'l']:
				numBytes = 4
			elif signalType in ['q', 'Q', 'd']:
				numBytes = 8
			else:
				raise Exception('received an invalid signal type in unpackBytes()')
			unpackedVariable = struct.unpack(f'={signalType}', packedBytes[0:numBytes])[0]
			packedBytes = packedBytes[numBytes:]
			return unpackedVariable, packedBytes

	def updateInternalVariables(self):
		self.totalSimulationTime = vsiCommonPythonApi.getTotalSimulationTime()
		self.stopRequested = vsiCommonPythonApi.isStopRequested()
		self.simulationStep = vsiCommonPythonApi.getSimulationStep()



def main():
	inputArgs = argparse.ArgumentParser(" ")
	inputArgs.add_argument('--domain', metavar='D', default='AF_UNIX', help='Socket domain for connection with the VSI TLM fabric server')
	inputArgs.add_argument('--server-url', metavar='CO', default='localhost', help='server URL of the VSI TLM Fabric Server')

	# Start of user custom code region. Please apply edits only within these regions:  Main method

	# End of user custom code region. Please don't edit beyond this point.

	args = inputArgs.parse_args()
                      
	visualizer = Visualizer(args)
	visualizer.mainThread()



if __name__ == '__main__':
    main()
