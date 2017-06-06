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
 @brief Test unit for PointDetection
'''
class Test_PointDetection(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.PointDetection import PointDetection
		self.Settings 		= Settings
		self.PointDetection = PointDetection
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_PointDetection(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		###### START TEST #####
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				for fn_frame, fn_slframe in left_frames:
					self.TestPointDetection(folder, fn_frame, fn_slframe)
		###########################

	def TestPointDetection(self, folder, fn_frame, fn_slframe):
		'''
		 @brief Test function for point detection unit.

		 @param folder Input folder 
		 @param fn_frame Frame filename without points.
		 @param fn_slframe Frame filename with points.
		'''
		import timeit
		import numpy as np
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, MatplotShow

		print '\n'
		print '#----------- TESTING POINT DETECTION   \t---------------#'
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
		timeout = timeit.default_timer() - delay
		print 'Delay for blob point detection: {0} sec, detected blobs: {1}'.format(timeout, len(point_kp))

		delay = timeit.default_timer()
		hough_frame, edgel_map_filtered, boundary_hough_lines = pointDet.GetBoundaryHoughLines(frame_un, delta_frame, point_kp, filtrate_edge_points=True, draw=False)
		timeout = timeit.default_timer() - delay
		print 'Delay for finding boundary edges (filtered) + lines: {0} sec'.format(timeout)

		timeout = timeit.default_timer() - total_delay
		print 'Total delay for downscaling + undistort + blob + hough lines + bounded lines: {0} sec'.format(timeout)

		delay = timeit.default_timer()
		hough_frame, edgel_map_unfiltered, boundary_hough_lines = pointDet.GetBoundaryHoughLines(frame_un, delta_frame, point_kp, filtrate_edge_points=False, draw=False)
		timeout = timeit.default_timer() - delay
		print 'Delay for finding boundary edges + lines: {0} sec'.format(timeout)

		# Compute with drawn figures for representation.
		hough_frame, edgel_map_filtered, boundary_hough_lines = pointDet.GetBoundaryHoughLines(frame_un, delta_frame, point_kp, filtrate_edge_points=True, draw=True)
		hough_frame, edgel_map_unfiltered, boundary_hough_lines = pointDet.GetBoundaryHoughLines(frame_un, delta_frame, point_kp, filtrate_edge_points=False, draw=True)

		kp_frame = pointDet.DrawKeypoints(hough_frame, point_kp, create_new_frame=True)

		touple_frames = []
		#touple_frames.append(('Frame', frame_un))
		#touple_frames.append(('SL Frame', sl_frame_un))
		#touple_frames.append(('Feature Points', delta_frame))
		#touple_frames.append(('Edgel Map', edgel_map_filtered))
		#touple_frames.append(('Bounded Lines', hough_frame))
		touple_frames.append(('Edgel Lines (filtered)', edgel_map_filtered))
		touple_frames.append(('Edgel Lines (unfiltered)', edgel_map_unfiltered))
		if not(self.CheckAllTests()):
			#MatplotShow([('Keypoints', kp_frame)], fn_frame+'_pointdetection_test', savefig_folder=self.savefig_folder+'pointdetection_test/', save_fig=self.save_figs, save_fig_only=self.save_figs_only)
			MatplotShow(touple_frames, fn_frame+'_pointdetection_test', savefig_folder=self.savefig_folder+'pointdetection_test/', default_n_cols=2, save_fig=self.save_figs, save_fig_only=self.save_figs_only, inlude_main_title_in_plot=False)
