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
 @brief Test unit for EdgeHeading
'''
class Test_EdgeHeading(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.imgProcessing.Heading import EdgeHeading
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.PointDetection import PointDetection
		self.Settings 		= Settings
		self.EdgeHeading 	= EdgeHeading
		self.PointDetection = PointDetection
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_EdgeHeading(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		###### START TEST #####
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				for fn_frame, fn_slframe in left_frames:
					self.TestEdgeHeading(folder, fn_frame, fn_slframe)
		###########################

	def TestEdgeHeading(self, folder, fn_frame, fn_slframe):
		'''
		 @brief Test function for EdgeHeading unit.

		 @param folder Input folder 
		 @param fn_frame Frame filename without points.
		 @param fn_slframe Frame filename with points.
		'''
		import timeit
		import numpy as np
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, MatplotShow

		print '\n'
		print '#----------- TESTING EDGE HEADING PROCESSING   \t---------------#'
		print '#----------- Image without points: {0} \t---------------#'.format(fn_frame)
		print '#----------- Image with points: {0}    \t---------------#'.format(fn_slframe)

		settings_inst 	= self.Settings.Settings()

		fn_frame 	= folder + fn_frame
		fn_slframe 	= folder + fn_slframe

		delay = timeit.default_timer()
		frame 		= GetImage(fn_frame)
		sl_frame 	= GetImage(fn_slframe)
		print 'Delay reading images: {0} sec'.format(timeit.default_timer() - delay)

		edgeHeading 	= self.EdgeHeading.EdgeHeading()

		pointDet 	= self.PointDetection.PointDetection(True, settings_inst.GetSettings())
		pointDet.CalibratePointDetection()
		print 'Min distance between blobs: {0}'.format(pointDet.GetMinDistanceBetweenBlobs())

		total_delay = timeit.default_timer()

		delay = timeit.default_timer()
		delta_frame, point_kp, blob_desc, frame_un, sl_frame_un = pointDet.GetPointList(frame, sl_frame, draw=True)
		print 'Delay for blob point detection: {0} sec, detected blobs: {1}'.format(timeit.default_timer() - delay, len(point_kp))

		delay = timeit.default_timer()
		hough_frame, edgel_map_filtered, boundary_hough_lines = pointDet.GetBoundaryHoughLines(frame_un, delta_frame, point_kp, draw=True, print_hough_positions=True)
		print 'Delay for finding boundary edges (filtered) + lines: {0} sec'.format(timeit.default_timer() - delay)

		delay = timeit.default_timer()
		selected_hor_edge_heading, selected_vert_edge_heading, possible_hor_edge_headings, possible_vert_edge_headings = edgeHeading.ComputeEdgeHeading(edgel_map_filtered, boundary_hough_lines, draw=False)
		print 'Delay for finding edge heading angle: {0} sec, hor_edge_heading = {1}, vert_edge_heading = {2}'.format(timeit.default_timer() - delay, selected_hor_edge_heading, selected_vert_edge_heading)

		timeout = timeit.default_timer() - total_delay
		print 'Total delay for downscaling + undistort + blob + hough lines + bounded lines + edge heading: {0} sec'.format(timeout)

		edgel_map_filtered_all_headings = np.array(edgel_map_filtered, dtype=edgel_map_filtered.dtype)
		selected_hor_edge_heading, selected_vert_edge_heading, possible_hor_edge_headings, possible_vert_edge_headings, edgel_map_filtered_all_headings = edgeHeading.ComputeEdgeHeading(edgel_map_filtered_all_headings, boundary_hough_lines, draw=True)

		touple_frames = []
		#touple_frames.append(('SL frame', sl_frame))
		#touple_frames.append(('SL undistorted', sl_frame_un))
		#touple_frames.append(('Original points', delta_frame))
		#touple_frames.append(('Hough lines', hough_frame))
		#touple_frames.append(('Selected edge heading', edgel_map_filtered))
		touple_frames.append(('Possible edge headings', edgel_map_filtered_all_headings))
		print 'max_hor = BLUE, min_hor = RED, max_vert = PURPLE, min_vert = GREEN'
		if not(self.CheckAllTests()):
			MatplotShow(touple_frames, fn_frame+'_Edge_heading_test', savefig_folder=self.savefig_folder+'edge_heading_test/', save_fig=self.save_figs, save_fig_only=self.save_figs_only, inlude_main_title_in_plot=False)