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
 @brief Test unit for detectEdges
'''
class Test_detectEdges(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.generalDetectors import detectEdges
		self.Settings 		= Settings
		self.detectEdges 	= detectEdges
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_detectEdges(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		###### START TEST #####
		print 'Edge detection is tested by Heading/EdgeHeading test'
		###########################