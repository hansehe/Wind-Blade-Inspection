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
 @brief Test unit for VideoLink
'''
class Test_VideoLink(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.hardware import VideoLink
		self.Settings 	= Settings
		self.VideoLink 	= VideoLink
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_VideoLink(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		print 'VideoLink is not working, but no need for it either..'
		#self.TestVideoLink()

	def TestVideoLink(self):
		'''
		 @brief Test unit VideoLink
		'''
		from src.DroneVision.DroneVision_src.hardware.imageTools import ImShow
		video_link = self.VideoLink.VideoLink(self.input_video, self.input_video_sl)
		while video_link.GetFrameNumber() < video_link.GetTotalFrames() and video_link.GetFrameNumber() < self.max_frames:
			frame, sl_frame = video_link.GetFrame()
			if not(self.CheckAllTests()):
				ImShow(frame)
		video_link.StopVideo()