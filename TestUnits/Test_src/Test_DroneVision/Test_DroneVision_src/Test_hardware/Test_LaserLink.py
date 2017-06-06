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

		laserlink.InitLaser()

		for i in range(0, 2):
			time.sleep(0.5)
			laserlink.LaserON()
			time.sleep(0.5)
			laserlink.LaserOFF()