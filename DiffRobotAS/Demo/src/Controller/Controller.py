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

		# Outputs
		self.v = 0
		self.omega = 0



srcMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBB]
SimulatorMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBA]
VisualizerMacAddress = [0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]
srcIpAddress = [192, 168, 1, 3]
SimulatorIpAddress = [192, 168, 1, 2]
VisualizerIpAddress = [192, 168, 1, 4]

SimulatorSocketPortNumber0 = 8070
ControllerSocketPortNumber1 = 8072

Controller0 = 0
Visualizer1 = 1


# Start of user custom code region. Please apply edits only within these regions:  Global Variables & Definitions
###########Adjustment of the PID controller values 
Kp=1.8
Kd=0.6
Ki=	0.1

import numpy as np

class PID:
    def __init__(self,Kp,Kd,Ki):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.U_prev = 0
        self.Integral = 0

    def Pid(self,U,dt):
        self.Integral += U*dt
        Derivative = (U-self.U_prev)/dt
        Y = self.Kp*U + self.Kd*Derivative + self.Ki*self.Integral
        self.U_prev =  	U
        return Y


class ControllerLogic:
	def __init__(self,Kp,Kd,Ki):
		self.pid = PID(Kp,Kd,Ki)
		self.v = 0
		self.omega = 0

  

	def Control(self, x, y, theta, x_ref, y_ref, dt):



		e_x = x_ref - x
		e_y = y_ref- y

		theta_ref = math.atan2(e_y, e_x)	
		

		
		e_theta = theta_ref - theta

	
		e_theta = math.atan2(math.sin(e_theta), math.cos(e_theta))

		self.omega = self.pid.Pid(e_theta, dt)

		# saturation
		self.omega = max(-5, min(5, self.omega))

	
		# i added these line to stop the motion and allow the robot to rotate on its axis instead of doing a circular movment
		if abs(e_theta) > 0.5:
			self.v = 0.0
		else:
			self.v = abs((1 - 0.5 * abs(e_y)) * math.exp(-abs(e_theta)))
			self.v = max(1.0, self.v)
			

		return self.v, self.omega
# End of user custom code region. Please don't edit beyond this point.
class Controller:

	def __init__(self, args):
		self.componentId = 1
		self.localHost = args.server_url
		self.domain = args.domain
		self.portNum = 50102
        
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
		self.controller = ControllerLogic(Kp,Kd,Ki)
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
				dt = self.simulationStep *1e-9

				v, omega = self.controller.Control(
				self.mySignals.x_robot,
				self.mySignals.y_robot,
				self.mySignals.theta,
				self.mySignals.x_path,
				self.mySignals.y_path,
				dt
				)

				self.mySignals.v = v
				self.mySignals.omega = omega	

				# End of user custom code region. Please don't edit beyond this point.
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

				receivedData = vsiEthernetPythonGateway.recvEthernetPacket(self.clientPortNum[Visualizer1])
				if(receivedData[3] != 0):
					self.decapsulateReceivedData(receivedData)

				# Start of user custom code region. Please apply edits only within these regions:  Before sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				#Send ethernet packet to Simulator
				self.sendEthernetPacketToSimulator()

				#Send ethernet packet to Visualizer
				self.sendEthernetPacketToVisualizer()

				# Start of user custom code region. Please apply edits only within these regions:  After sending the packet

				# End of user custom code region. Please don't edit beyond this point.

				print("\n+=Controller+=")
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
				print("  Outputs:")
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
			self.clientPortNum[Controller0] = vsiEthernetPythonGateway.tcpConnect(bytes(SimulatorIpAddress), SimulatorSocketPortNumber0)

		if(self.clientPortNum[Visualizer1] == 0):
			self.clientPortNum[Visualizer1] = vsiEthernetPythonGateway.tcpListen(ControllerSocketPortNumber1)

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


	def sendEthernetPacketToSimulator(self):
		bytesToSend = bytes()

		bytesToSend += self.packBytes('d', self.mySignals.v)

		bytesToSend += self.packBytes('d', self.mySignals.omega)

		#Send ethernet packet to Simulator
		vsiEthernetPythonGateway.sendEthernetPacket(SimulatorSocketPortNumber0, bytes(bytesToSend))

	def sendEthernetPacketToVisualizer(self):
		bytesToSend = bytes()

		bytesToSend += self.packBytes('d', self.mySignals.v)

		bytesToSend += self.packBytes('d', self.mySignals.omega)

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
                      
	controller = Controller(args)
	controller.mainThread()



if __name__ == '__main__':
    main()
