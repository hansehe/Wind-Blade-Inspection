'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

################### UNIT TEST ########################
import unittest

from Test_MasterSubClass import Test_MasterSubClass
'''
 @brief Test unit for Master
'''
class Test_Master(unittest.TestCase, Test_MasterSubClass):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.InitMasterTest()

	def test_StartMasterTest(self):
		'''
		 @brief Main start test function.
		'''
		import sys
		###### START TEST #####
		if self.CheckAllTests() or 'Slave' in sys.argv:
			self.TestMasterSlave()
		else:
			self.TestMaster()
		###########################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

