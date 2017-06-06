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
 @brief Test unit for ImageLink
'''
class Test_ImageLink(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.hardware import ImageLink
		self.Settings 	= Settings
		self.ImageLink 	= ImageLink
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def GetConfiguredSettings(self, folder, left_frames, right_frames):
		'''
		 @brief Get configured settings with correct input images/videos

		 @param folder
		 @param left_frames
		 @param right_frames

		 @return settings
		'''
		from TestUnits.Test_src.Test_DroneMasterSubClass import Test_DroneMasterSubClass
		test_DroneMaster = Test_DroneMasterSubClass()
		return test_DroneMaster.GetConfiguredSettings(folder, left_frames, right_frames) # Follow settings syntax from DroneMaster test

	def test_ImageLink(self):
		'''
		 @brief Test unit for the image link
		'''
		from src.DroneVision.DroneVision_src.hardware.imageTools import ImShow
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				settings_inst 	= self.GetConfiguredSettings(folder, left_frames, right_frames)
				imagelink 		= self.ImageLink.ImageLink(settings_inst.GetSettings('IMAGE', 'input_folder'), settings_inst.GetSettings('IMAGE', 'right_images'), settings_inst.GetSettings('IMAGE', 'right_sl_images'))
				for i in range(len(settings_inst.GetSettings('IMAGE', 'right_sl_images'))):
					frame, sl_frame = imagelink.GetFrame()
					if not(self.CheckAllTests()):
						ImShow(frame, 'Testing ImageLink')
				break #needs only a single test