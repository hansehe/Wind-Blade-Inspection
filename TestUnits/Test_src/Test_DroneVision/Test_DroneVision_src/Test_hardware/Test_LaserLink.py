'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

################### UNIT TEST ########################
import unittest

from Settings.TestData import TestData
from TestUnits.Test_main import Test_main
'''
 @brief Test unit for LaserLink
'''
class Test_LaserLink(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		TestData.__init__(self)
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.hardware import LaserLink
		self.Settings 	= Settings
		self.LaserLink 	= LaserLink
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_LaserLink(self):
		'''
		 @brief Test laser by turning it on and off
		'''
		import time

		settings_inst  = self.Settings.Settings()
		laserlink = self.LaserLink.LaserLink(settings_inst.GetSettings('LASER', 'laser_triggerPin'))

		laserlink.AssertAvailableLaser()

		for i in range(0, 10):
			time.sleep(0.1)
			laserlink.LaserON()
			time.sleep(0.1)
			laserlink.LaserOFF()

	def test_LaserLinkManualTrig(self):
		'''
		 @brief Test laser by turning it on and off manually
		'''
		if self.CheckAllTests():
			return

		import time
		from getpass import getpass
		from src.bin.UserInput.UserInput import UserInput

		settings_inst  = self.Settings.Settings()
		laserlink = self.LaserLink.LaserLink(settings_inst.GetSettings('LASER', 'laser_triggerPin'))
		userInput = UserInput(settings_inst.GetSettings('USER_INPUT'))

		laserlink.AssertAvailableLaser()

		while (not(userInput.CheckTerminated())):
			getpass('Hit enter to turn laser ON..')
			laserlink.LaserON()
			getpass('Hit enter to turn laser OFF..')
			laserlink.LaserOFF()