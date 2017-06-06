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
 @brief Test unit for PyQtImage
'''
class Test_PyQtImage(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from src.DroneVision.DroneVision_src.hardware import PyQtImage
		self.PyQtImage = PyQtImage
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_PyQtGraphImageSingleImage(self):
		'''
		 @brief Test PyQtImage real-time plot with single image
		'''
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage
		import time, timeit
		from getpass import getpass

		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				self.pqImage = self.PyQtImage.PyQtImage(initialize=True, title='Test real-time single plot')
				timer = timeit.default_timer()
				for left_frame in left_frames:
					input_image_fn = left_frame[0]
					frame = GetImage(folder+input_image_fn)
					self.pqImage.UpdatePQImage(frame, 'left_frame', rotate=True)
					#if not(self.CheckAllTests()):
					#	getpass('Press enter to continue to next frame..')
				print 'Delay for processing {0} images was: {1} sec'.format(len(left_frames), timeit.default_timer()-timer)
				self.pqImage.ClosePQWindow()

	def test_PyQtGraphImageMultipleImages(self):
		'''
		 @brief Test PyQtImage real-time plot with multiple images
		'''
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage
		from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import CheckColor, GetShape
		import time, cv2, timeit
		from getpass import getpass

		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				self.pqImage = self.PyQtImage.PyQtImage(initialize=True, title='Test real-time multiple plot')
				timer = timeit.default_timer()
				reset = True
				for i in range(len(left_frames)):
					frame_l 	= GetImage(folder+left_frames[i][0])
					frame_l_sl 	= GetImage(folder+left_frames[i][1])
					frame_r 	= GetImage(folder+right_frames[i][0])
					frame_r_sl 	= GetImage(folder+right_frames[i][1])
					frame_l 		= CheckColor(frame_l)
					widht, height 	= GetShape(frame_l)
					cv2.line(frame_l, (0,0), (height,widht), (255,255,0), 10)
					touple_frames = [('frame_l', frame_l), ('frame_l_sl', frame_l_sl), ('frame_r', frame_r), ('frame_r_sl', frame_r_sl)]
					self.pqImage(touple_frames, rotate=True)
					if not(self.CheckAllTests()):
						getpass('Press enter to continue to next frame..')
					if reset:
						self.pqImage(reset=True)
						reset = False
					else:
						reset = True
				print 'Delay for processing {0} image sets was: {1} sec'.format(len(left_frames), timeit.default_timer()-timer)
				self.pqImage.ClosePQWindow()


