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
 @brief Test unit for BlobScaleDetector
'''
class Test_BlobScaleDetector(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.BlobScaleDetector import BlobScaleDetector
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.PointDetection import PointDetection
		self.Settings 			= Settings
		self.BlobScaleDetector 	= BlobScaleDetector
		self.PointDetection 	= PointDetection
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_BlobScaleDetector(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		###### START TEST #####
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				for i in range(len(left_frames)):
					self.TestBlobScaleDetector(folder, left_frames[i][0], left_frames[i][1], actual_distances[i])
		###########################

	def TestBlobScaleDetector(self, folder, fn_frame, fn_slframe, actual_distance):
		'''
		 @brief Test function for blob scale detector unit.

		 @param folder Input folder
		 @param fn_frame Frame filename without points.
		 @param fn_slframe Frame filename with points.
		 @param actual_distance (mm)
		'''
		import timeit, math
		import numpy as np
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, MatplotShow
		from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape

		print '\n'
		print '#----------- TESTING SCALING BASED BLOB DETECTION   \t---------------#'
		print '#----------- Image without points: {0} \t---------------#'.format(fn_frame)
		print '#----------- Image with points: {0}    \t---------------#'.format(fn_slframe)

		settings_inst 	= self.Settings.Settings()
		settings_inst.ChangeSetting('BASIC', 'reset_calibration', False)
		pointDet 		= self.PointDetection.PointDetection(True, settings_inst.GetSettings())

		pointDet.CalibrateBlobScaleDetector(printInfo=True, force_calibration=True)
		print 'Min distance between blobs: {0}'.format(pointDet.GetMinDistanceBetweenBlobs())

		fn_frame 	= folder + fn_frame
		fn_slframe 	= folder + fn_slframe

		delay = timeit.default_timer()
		frame 		= GetImage(fn_frame)
		sl_frame 	= GetImage(fn_slframe)
		print 'Delay reading images: {0} sec'.format(timeit.default_timer() - delay)

		total_delay = timeit.default_timer()

		delta_frame, points_kp, blob_desc, frame, sl_frame = pointDet.GetPointList(frame, sl_frame, concatenate_points=True, draw=True)

		delay = timeit.default_timer()
		width, height 	= GetShape(sl_frame)
		blockSize 		= int(math.sqrt(math.pow(width, 2.0) + math.pow(height, 2.0))//4)
		scaling_point_frame, mean_point_size = pointDet.PointScaleMatch(delta_frame, points_kp, blockSize)
		print 'Delay for computing disparity points: {0} sec'.format(timeit.default_timer() - delay)

		timeout = timeit.default_timer() - total_delay
		print 'Total delay for downscaling + undistort + blob + feature based stereopsis: {0} sec'.format(timeout)

		touple_frames = []
		touple_frames.append(('SL frame', sl_frame))
		touple_frames.append(('Delta frame', delta_frame))
		if not(self.CheckAllTests()):
			MatplotShow(touple_frames, fn_frame+'_Blob_scale_test', save_fig=self.save_figs, save_fig_only=self.save_figs_only)