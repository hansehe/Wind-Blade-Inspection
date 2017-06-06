'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

################### UNIT TEST ########################

import unittest

from Test_DroneSlaveSubClass import Test_DroneSlaveSubClass
'''
 @brief Test unit for DroneSlave
'''
class Test_DroneSlave(unittest.TestCase, Test_DroneSlaveSubClass):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.InitDroneSlaveTest()

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_DroneSlave(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		import sys

		###### START TEST #####
		if not(self.CheckAllTests() or 'DroneMaster' in sys.argv):
			self.TestDroneSlave(print_progress=True)
		else:
			print 'DroneSlave is tested by the DroneMaster test'
		###########################


