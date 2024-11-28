from Phidget22.Phidget import *
from Phidget22.Devices.VoltageInput import *
from Phidget22.Devices.DigitalInput import *


def onVoltageChange(self, voltage):
	global ai0
	ai0 = voltage


def onStateChange(self, state):
	global di0, di1
	if self.getChannel() == 0:
		di0 = state
	elif self.getChannel() == 1:
		di1 = state


class PhidgetDIO:
	def __init__(self):
		self.ai0_handle = VoltageInput()
		self.di0_handle = DigitalInput()
		self.di1_handle = DigitalInput()
		self.__ai0_state = None
		self.__di0_state = None
		self.__di1_state = None

		self.ai0_handle.setDeviceSerialNumber(441105)
		self.di0_handle.setDeviceSerialNumber(441105)
		self.di0_handle.setChannel(0)
		self.di1_handle.setDeviceSerialNumber(441105)
		self.di1_handle.setChannel(1)

		self.ai0_handle.setOnVoltageChangeHandler(onVoltageChange)
		self.di0_handle.setOnStateChangeHandler(onStateChange)
		self.di1_handle.setOnStateChangeHandler(onStateChange)

		self.ai0_handle.openWaitForAttachment(5000)
		self.di0_handle.openWaitForAttachment(5000)
		self.di1_handle.openWaitForAttachment(5000)
		self.ai0_handle.setDataRate(DataRate=200)		# Hz

	def __del__(self):
		self.ai0_handle.close()
		self.di0_handle.close()
		self.di1_handle.close()

	def get_di0(self):
		self.__di0_state = di0
		return self.__di0_state

	def get_di1(self):
		self.__di1_state = di1
		return self.__di1_state

	def get_ai0(self):
		self.__ai0_state = ai0
		return self.__ai0_state


if __name__ == "__main__":
	import time
	phidget = PhidgetDIO()
	while True:
		print (phidget.get_di0(), phidget.get_di1(), phidget.get_ai0())
		time.sleep(0.01)