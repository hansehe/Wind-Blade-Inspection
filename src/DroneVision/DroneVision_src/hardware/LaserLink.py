'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

import time
from PinControl import PinControl

'''
 @brief Class for handling the laser

 @param laser_triggerPin
'''
class LaserLink():
	def __init__(self, laser_triggerPin):
		'''CONSTRUCTOR'''
		self.__laser_pin = PinControl(laser_triggerPin)
		if self.CheckAvailableLaser():
			self.InitLaser()

	def InitLaser(self):
		'''
		 @brief Initialize laser
		'''
		self.AssertAvailableLaser()

	def LaserON(self, wait_ready_t=0.01):
		'''
		 @brief Turn laser OFF

		 @param wait_ready_t (Time to wait for the laser to turn on. (default=0.01))
		'''
		self.AssertAvailableLaser()
		self.__laser_pin.SetPinHigh()
		time.sleep(wait_ready_t)

	def LaserOFF(self):
		'''
		 @brief Turn laser ON
		'''
		self.AssertAvailableLaser()
		self.__laser_pin.SetPinLow()

	def ToggleLaser(self, pause=1.0):
		'''
		 @brief Toggle laser

		 @param pause (delay between high->low (default=1.0 sec))
		'''
		self.LaserON()
		if pause > 0:
			time.sleep(pause)
		self.LaserOFF()

	def CheckAvailableLaser(self):
		'''
		 @brief Check if laser is enabled (laser_triggerPin >= 0)

		 @return True/False
		'''
		ok = False
		if self.__laser_pin.GetPin() >= 0:
			ok = True
		return ok

	def AssertAvailableLaser(self):
		'''
		 @brief Assert available laser
		'''
		if not(self.CheckAvailableLaser()):
			raise Exception('Laser is not available!')