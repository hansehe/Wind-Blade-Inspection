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
 @brief Test unit for BlobDetector
'''
class Test_BlobDetector(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection import BlobDetector
		self.Settings 		= Settings
		self.BlobDetector 	= BlobDetector
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_BlobDetector(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		###### START TEST #####
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				for fn_frame, fn_slframe in left_frames:
					self.TestBlobDetection(folder, fn_frame, fn_slframe)
		###########################

	def TestBlobDetection(self, folder, fn_frame, fn_slframe):
		'''
		 @brief Test function for blob detection unit.

		 @param folder Input folder 
		 @param fn_frame Frame filename without points.
		 @param fn_slframe Frame filename with points.
		'''
		import timeit, warnings
		import numpy as np
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, MatplotShow
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.PointDetection import PointDetection
		from Settings.Exceptions import DroneVisionError

		print '\n'
		print '#----------- TESTING BLOB DETECTION   \t---------------#'
		print '#----------- Image without points: {0} \t---------------#'.format(fn_frame)
		print '#----------- Image with points: {0}    \t---------------#'.format(fn_slframe)

		settings_inst 	= self.Settings.Settings()

		fn_frame 	= folder + fn_frame
		fn_slframe 	= folder + fn_slframe

		delay = timeit.default_timer()
		frame 		= GetImage(fn_frame)
		sl_frame 	= GetImage(fn_slframe)
		print 'Delay reading images: {0} sec'.format(timeit.default_timer() - delay)

		# Detector types: 0,1,2,3 - simple blob detector, ORB, SIFT, SURF
		detector_type_titles = ['Simple Blob Detector', 'ORB Detector', 'SIFT Detector', 'SURF Detector']
		feature_point_frames = []
		for detector_type in range(len(detector_type_titles)):

			settings_inst.ChangeSetting('CV', 'detector_type', detector_type)
			pointDet 	= PointDetection.PointDetection(True, settings_inst.GetSettings())
			pointDet.CalibratePointDetection()
			print '\n\nMin distance between blobs: {0}'.format(pointDet.GetMinDistanceBetweenBlobs())

			try:
				total_delay = timeit.default_timer()
				delta_frame, point_kp, blob_desc, frame_un, sl_frame_un = pointDet.GetPointList(frame, sl_frame, draw=False)
				timeout = timeit.default_timer() - total_delay
				print 'Total delay for downscaling + undistort + blob using {0}: {1} sec'.format(detector_type_titles[detector_type], timeout)
			except DroneVisionError, err:
				warnings.simplefilter('always')
				warnings.warn(str(err)+' - Detector type: {0}'.format(detector_type_titles[detector_type]), Warning)
				warnings.simplefilter('default')
				continue

			delta_frame_drawn, point_kp, blob_desc, frame_un, sl_frame_un = pointDet.GetPointList(frame, sl_frame, draw=True)
			feature_point_frames.append((detector_type_titles[detector_type], delta_frame_drawn))

		touple_frames = []
		touple_frames.append(('Frame', frame_un))
		touple_frames.append(('SL Frame', sl_frame_un))
		touple_frames.append(('Delta Frame', delta_frame))
		if not(self.CheckAllTests()):
			MatplotShow(touple_frames, fn_frame+'_Feature Points', default_n_cols=3, save_fig=self.save_figs, save_fig_only=self.save_figs_only)
			MatplotShow(feature_point_frames, fn_frame+'_Detected Feature Points', default_n_cols=4, save_fig=self.save_figs, save_fig_only=self.save_figs_only)