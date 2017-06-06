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
 @brief Test unit for PinControl
'''
class Test_PinControl(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.hardware import PinControl
		self.Settings 	= Settings
		self.PinControl = PinControl
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_PinControl(self):
		'''
		 @brief Test the PinControl class
		'''
		import timeit
		from src.bin.UserInput.UserInput import UserInput
		settings 			= self.Settings.Settings()
		pinControl_camera 	= self.PinControl.PinControl(settings.GetSettings('CAMERA', 'camera_triggerPin'))
		pinControl_laser 	= self.PinControl.PinControl(settings.GetSettings('LASER', 'laser_triggerPin'))

		settings.ChangeSetting('USER_INPUT', 'automatic_mode', True)
		if pinControl_camera.CheckPinControlAVailable():
			userInput = UserInput(settings.GetSettings('USER_INPUT'))
			print 'Testing camera pin on: {0}, and laser pin on: {1}'.format(pinControl_camera.GetPin(), pinControl_laser.GetPin())
			print 'TYPE q TO STOP TEST'
			timeout = timeit.default_timer()
			while (not(userInput.CheckTerminated())):
				if timeit.default_timer() - timeout > self.test_delay and self.CheckAllTests():
					break
				pinControl_laser.TogglePin()
				pinControl_camera.TogglePin()
			userInput.ForceTermination()

	def test_ManualPinToggel(self):
		'''
		 @brief Test manual pin toggling
		'''
		import time, sys
		from getpass import getpass
		from src.bin.UserInput.UserInput import UserInput
		settings 			= self.Settings.Settings()
		pinControl_camera 	= self.PinControl.PinControl(settings.GetSettings('CAMERA', 'camera_triggerPin'))
		pinControl_laser 	= self.PinControl.PinControl(settings.GetSettings('LASER', 'laser_triggerPin'))

		settings.ChangeSetting('USER_INPUT', 'automatic_mode', False)
		if pinControl_camera.CheckPinControlAVailable():
			if not(self.CheckAllTests()):
				userInput = UserInput(settings.GetSettings('USER_INPUT'))
				print 'Testing camera pin on: {0}, and laser pin on: {1}'.format(pinControl_camera.GetPin(), pinControl_laser.GetPin())
				print 'TYPE q TO STOP TEST'
				while (not(userInput.CheckTerminated())):
					userInput.PauseUserInput()
					getpass('Type enter to toggle pins...')
					userInput.RestartUserInput()
					pinControl_laser.TogglePin()
					pinControl_camera.TogglePin()
					#time.sleep(0.5)
				userInput.ForceTermination()
