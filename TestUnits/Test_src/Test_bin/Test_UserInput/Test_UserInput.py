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
 @brief Test unit for UserInput
'''
class Test_UserInput(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.bin.UserInput import UserInput
		self.Settings 	= Settings
		self.UserInput	= UserInput
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_UserInput(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		settings = self.Settings.Settings()
		settings.ChangeSetting('USER_INPUT', 'automatic_mode', True)
		userInput = self.UserInput.UserInput(settings.GetSettings('USER_INPUT'))
		if not(self.CheckAllTests()):
			print 'TYPE q to TERMINATE'
			while (not(userInput.CheckTerminated())):
				pass
			print 'TERMINATED'
		else:
			userInput.ForceTermination()