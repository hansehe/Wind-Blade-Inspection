'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

################### UNIT TEST ########################
import unittest

from Test_DroneMasterSubClass import Test_DroneMasterSubClass
'''
 @brief Test unit for DroneMaster
'''
class Test_DroneMaster(unittest.TestCase, Test_DroneMasterSubClass):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.InitDroneMasterTest()

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_DroneMaster(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		import sys

		###### START TEST #####
		if self.CheckAllTests() or 'DroneSlave' in sys.argv:
			self.TestDroneMasterSlave()
		else:
			self.TestDroneMaster()
		###########################


