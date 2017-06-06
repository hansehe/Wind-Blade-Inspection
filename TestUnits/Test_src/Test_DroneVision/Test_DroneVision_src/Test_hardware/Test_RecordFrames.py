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
 @brief Test unit for RecordFrames
'''
class Test_RecordFrames(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.hardware import RecordFrames
		self.Settings 		= Settings
		self.RecordFrames 	= RecordFrames
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_RecordFrames(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		###### START TEST #####
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				self.TestRecordFrames(folder + left_frames[0][0])
		###########################

	def TestRecordFrames(self, input_image_fn):
		'''
		 @brief Test unit for RecordFrames

		 @param input_image_fn
		'''
		import time
		from src.bin.tools import CheckDir
		CheckDir(self.vid_rec_folder)
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage
		from src.DroneVision.DroneVision_src.hardware.VideoLink import VideoLink
		video_recorder 	= self.RecordFrames.RecordFrames(self.vid_rec_fps, self.vid_rec_folder, self.video_rec_output_fname)
		while video_recorder.GetNumberOfRecordedFrames() < self.max_rec_frames:
			frame = GetImage(input_image_fn)
			#video_recorder.WriteFrameThread(frame)
			#time.sleep(0.5)
			video_recorder.WriteFrame(frame)
			print 'N recorded frames: ', video_recorder.GetNumberOfRecordedFrames()
		video_recorder.CloseRecording()


		