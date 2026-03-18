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
		self.v = 0
		self.omega = 0

		# Outputs
		self.x_robot = 0
		self.y_robot = 0
		self.theta = 0
		self.x_path = 0
		self.y_path = 0
		self.x_path_curr = 0
		self.y_path_curr = 0



srcMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBA]
ControllerMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBB]
VisualizerMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]
srcIpAddress = [192, 168, 1, 2]
ControllerIpAddress = [192, 168, 1, 3]
VisualizerIpAddress = [192, 168, 1, 4]

SimulatorSocketPortNumber0 = 8070
SimulatorSocketPortNumber1 = 8071

Controller0 = 0
Visualizer1 = 1


# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions
# Noise configuration
noise_level = "none"  # none /low / medium / high
sigma_pos = 0.01  # Standard deviation for position noise
sigma_theta = 0.005  # Standard deviation for orientation noise
flag =True
flag2=True
import numpy as np

class Robot:
    def __init__(self,x,y,theta):
        self.__x=x
        self.__y=y
        self.__theta=theta

    def get_pos(self):
        return self.__x,self.__y,self.__theta

    def set_pos(self,x,y):
        self.__x=x
        self.__y=y

    def move(self,v,omega,dt):
        self.__x = self.__x + v*np.cos(self.__theta)*dt
        self.__y = self.__y + v*np.sin(self.__theta)*dt
        self.__theta = self.__theta + omega*dt


class Path:

    def __init__(self,ln):
        self.path_length=ln

    def straight_Path(self,x):
        if x < self.path_length:
            return 1
        else:
            return None

    def Curved_Path(self,x):
        if x < self.path_length:
            return np.sin(x)
        else:
            return None
# End of user custom code region. Please don't edit beyond this point.
class Simulator:

	def __init__(self, args):
		self.componentId = 0
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50101
        
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
		self.path_type ="straight"

		self.robot = Robot(
		np.random.uniform(0,2),
		np.random.uniform(-2,2),
		np.random.rand()*2*np.pi
		)

		self.path = Path(300)
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
				dt = self.simulationStep*1e-9
				global flag2
				if noise_level == "none":
					sigma_pos = 0
					sigma_theta = 0
					if flag2:
						print("Ideal mode")
				elif noise_level == "low":
					sigma_pos = 0.005
					sigma_theta = 0.002
					if flag2:
						print("Low noise mode")
				elif noise_level == "medium":
					sigma_pos = 0.010
					sigma_theta = 0.005
					if flag2:
						print("Medium noise mode")
				elif noise_level == "high":
					sigma_pos = 0.050
					sigma_theta = 0.020
					if flag2:
						print("High noise mode")
				flag2 =False
				# move robot
				self.robot.move(self.mySignals.v, self.mySignals.omega, dt)
				x, y, theta = self.robot.get_pos()
				global flag
				if flag:
					print(f"Initial position: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
					flag=0
				l=1.0
				x_ref=x+l
				#i added the current  signal for plotting while the look head set for the calculations
				x_ref_curr=x
				# path calculation
				if self.path_type == "straight":
					y_ref = self.path.straight_Path(x_ref)
					y_ref_curr=self.path.straight_Path(x_ref_curr)
				else:
					y_ref = self.path.Curved_Path(x_ref)
					y_ref_curr=self.path.Curved_Path(x_ref_curr)

			

				if y_ref is None or y_ref_curr is None:
					y_ref = 0
					y_ref_curr=0

				# add noise	
				noise_x = np.random.normal(0, sigma_pos)
				noise_y = np.random.normal(0, sigma_pos)
				noise_theta = np.random.normal(0, sigma_theta)

				x = x + noise_x
				y = y + noise_y
				theta = theta + noise_theta
				# send signals to controller and visualizer
				self.mySignals.x_robot = x
				self.mySignals.y_robot = y
				self.mySignals.theta = theta
				self.mySignals.x_path = x_ref
				self.mySignals.y_path = y_ref
				self.mySignals.x_path_curr=x_ref_curr
				self.mySignals.y_path_curr=y_ref_curr
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

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(self.clientPortNum[Controller0])
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(self.clientPortNum[Visualizer1])
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				#Send ethernet packet to Controller
				self.sendEthernetPacketToController()

				#Send ethernet packet to Visualizer
				self.sendEthernetPacketToVisualizer()

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=Simulator+=")
				print("  VSI time:", end = " ")
				print(vsiCommonPythonApi.getSimulationTimeInNs(), end = " ")
				print("ns")
				print("  Inputs:")
				print("\tv =", end = " ")
				print(self.mySignals.v)
				print("\tomega =", end = " ")
				print(self.mySignals.omega)
				print("  Outputs:")
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

			if(vsiCommonPythonApi.getSimulationTimeInNs() < self.totalSimulationTime):
				vsiEthernetPythonGateway.terminate()
		except Exception as e:
			if str(e) == "stopRequested":
				print("Terminate signal has been received from one of the VSI clients")
				# Advance time with a step that is equal to "simulationStep + 1" so that all other clients
				# receive the terminate packet before terminating this client
				vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)
			else:
				print(f"An error occurred: {str(e)}")
		except:
			# Advance time with a step that is equal to "simulationStep + 1" so that all other clients
			# receive the terminate packet before terminating this client
			vsiCommonPythonApi.advanceSimulation(self.simulationStep + 1)



	def establishTcpUdpConnection(self):
		if(self.clientPortNum[Controller0] == 0):
			self.clientPortNum[Controller0] = vsiEthernetPythonGateway.tcpListen(SimulatorSocketPortNumber0)

		if(self.clientPortNum[Visualizer1] == 0):
			self.clientPortNum[Visualizer1] = vsiEthernetPythonGateway.tcpListen(SimulatorSocketPortNumber1)

		if(self.clientPortNum[Visualizer1] == 0):
			print("Error: Failed to connect to port: Simulator on TCP port: ") 
			print(SimulatorSocketPortNumber0)
			exit()

		if(self.clientPortNum[Visualizer1] == 0):
			print("Error: Failed to connect to port: Simulator on TCP port: ") 
			print(SimulatorSocketPortNumber1)
			exit()



	def decapsulateReceivedData(self, receivedData):
		self.receivedDestPortNumber = receivedData[0]
		self.receivedSrcPortNumber = receivedData[1]
		self.receivedNumberOfBytes = receivedData[3]
		self.receivedPayload = [0] * (self.receivedNumberOfBytes)

		for i in range(self.receivedNumberOfBytes):
			self.receivedPayload[i] = receivedData[2][i]

		if(self.receivedSrcPortNumber == self.clientPortNum[Controller0]):
			print("Received packet from Controller")
			receivedPayload = bytes(self.receivedPayload)
			self.mySignals.v, receivedPayload = self.unpackBytes('d', receivedPayload)

			self.mySignals.omega, receivedPayload = self.unpackBytes('d', receivedPayload)


	def sendEthernetPacketToController(self):
		bytesToSend = bytes()

		bytesToSend += self.packBytes('d', self.mySignals.x_robot)

		bytesToSend += self.packBytes('d', self.mySignals.y_robot)

		bytesToSend += self.packBytes('d', self.mySignals.theta)

		bytesToSend += self.packBytes('d', self.mySignals.x_path)

		bytesToSend += self.packBytes('d', self.mySignals.y_path)

		#Send ethernet packet to Controller
		vsiEthernetPythonGateway.sendEthernetPacket(self.clientPortNum[Controller0], bytes(bytesToSend))

	def sendEthernetPacketToVisualizer(self):
		bytesToSend = bytes()

		bytesToSend += self.packBytes('d', self.mySignals.x_robot)

		bytesToSend += self.packBytes('d', self.mySignals.y_robot)

		bytesToSend += self.packBytes('d', self.mySignals.theta)

		bytesToSend += self.packBytes('d', self.mySignals.x_path)

		bytesToSend += self.packBytes('d', self.mySignals.y_path)

		bytesToSend += self.packBytes('d', self.mySignals.x_path_curr)

		bytesToSend += self.packBytes('d', self.mySignals.y_path_curr)

		#Send ethernet packet to Visualizer
		vsiEthernetPythonGateway.sendEthernetPacket(self.clientPortNum[Visualizer1], bytes(bytesToSend))

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
                      
	simulator = Simulator(args)
	simulator.mainThread()



if __name__ == '__main__':
    main()
