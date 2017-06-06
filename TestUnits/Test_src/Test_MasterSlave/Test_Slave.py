'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

################### UNIT TEST ########################
import unittest

from Test_SlaveSubClass import Test_SlaveSubClass
'''
 @brief Test unit for Master
'''
class Test_Slave(unittest.TestCase, Test_SlaveSubClass):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.InitSlaveTest()

	def test_StartSlaveTest(self):
		'''
		 @brief Main start test function.
		'''
		import sys
		###### START TEST #####
		if not(self.CheckAllTests() or 'Master' in sys.argv):
			self.TestSlave()
		else:
			print 'Slave is tested by the Master test'
		###########################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass