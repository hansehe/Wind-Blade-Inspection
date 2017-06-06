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
 @brief Test unit for detectCorners
'''
class Test_detectCorners(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.PointDetection import PointDetection
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.generalDetectors import detectCorners
		self.Settings 		= Settings
		self.PointDetection = PointDetection
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_detectLines(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		###### START TEST #####
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				for fn_frame, fn_slframe in left_frames:
					self.TestCornerDetection(folder, fn_frame, fn_slframe)
		###########################

	def TestCornerDetection(self, folder, fn_frame, fn_slframe):
		'''
		 @brief Test function for corner detection unit.

		 @param folder Input folder 
		 @param fn_frame Frame filename without points.
		 @param fn_slframe Frame filename with points.
		'''
		import timeit
		import numpy as np
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, MatplotShow
		from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.generalDetectors.detectEdges import Canny
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.generalDetectors.detectCorners import DetectCorners, DrawCorners, DetectCornersCV2

		print '\n'
		print '#----------- TESTING CORNER DETECTION   \t---------------#'
		print '#----------- Image without points: {0} \t---------------#'.format(fn_frame)
		print '#----------- Image with points: {0}    \t---------------#'.format(fn_slframe)

		settings_inst 	= self.Settings.Settings()

		fn_frame 	= folder + fn_frame
		fn_slframe 	= folder + fn_slframe

		delay = timeit.default_timer()
		frame 		= GetImage(fn_frame)
		sl_frame 	= GetImage(fn_slframe)
		print 'Delay reading images: {0} sec'.format(timeit.default_timer() - delay)

		pointDet 	= self.PointDetection.PointDetection(True, settings_inst.GetSettings())
		pointDet.CalibratePointDetection()
		print 'Min distance between blobs: {0}'.format(pointDet.GetMinDistanceBetweenBlobs())

		total_delay = timeit.default_timer()

		delay = timeit.default_timer()
		delta_frame, point_kp, blob_desc, frame_un, sl_frame_un = pointDet.GetPointList(frame, sl_frame, draw=False)
		print 'Delay for blob point detection: {0} sec, detected blobs: {1}'.format(timeit.default_timer() - delay, len(point_kp))
		
		delay = timeit.default_timer()
		corners = DetectCorners(delta_frame, point_kp, kernel_size=pointDet.GetMinDistanceBetweenBlobs()*6, kernel_step_size=pointDet.GetMinDistanceBetweenBlobs(), k=0.06, thresh=1000)
		timeout = timeit.default_timer() - delay
		print 'Delay for finding corners: {0} sec, (shape = {1}), number of corners: {2}'.format(timeout, delta_frame.shape, len(corners))

		delay = timeit.default_timer()
		corners_cv2 = DetectCornersCV2(delta_frame, point_kp, kernel_size=int(pointDet.GetMinDistanceBetweenBlobs()*6), k_sobel_size=3, k=0.04, thresh=0.8)
		timeout = timeit.default_timer() - delay
		print 'Delay for finding corners (cv2): {0} sec, (shape = {1}), number of corners: {2}'.format(timeout, delta_frame.shape, len(corners_cv2))

		delay = timeit.default_timer()
		corners_cv2_canny 	= DetectCornersCV2(Canny(frame_un), [], kernel_size=2, k_sobel_size=3, k=0.04, thresh=0.8)
		timeout = timeit.default_timer() - delay
		print 'Delay for finding corners (cv2): {0} sec, (shape = {1}), number of corners: {2}'.format(timeout, delta_frame.shape, len(corners_cv2))

		corners_frame = np.array(delta_frame)
		corners_frame = DrawCorners(corners_frame, corners)

		corners_frame_cv2 	= np.array(delta_frame)
		corners_frame_cv2 	= DrawCorners(corners_frame_cv2, corners_cv2)

		corners_frame_cv2_canny = np.array(Canny(frame_un))
		corners_frame_cv2_canny = DrawCorners(corners_frame_cv2_canny, corners_cv2_canny)

		touple_frames = []
		touple_frames.append(('Corners', corners_frame))
		touple_frames.append(('Corners (cv2)', corners_frame_cv2))
		touple_frames.append(('Corners Canny (cv2)', corners_frame_cv2_canny))
		touple_frames.append(('Delta Frame', delta_frame))
		if not(self.CheckAllTests()):
			MatplotShow(touple_frames, fn_frame+'_corner_test', save_fig=self.save_figs, save_fig_only=self.save_figs_only)