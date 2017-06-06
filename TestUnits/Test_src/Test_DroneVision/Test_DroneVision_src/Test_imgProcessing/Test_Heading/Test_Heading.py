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
 @brief Test unit for Heading
'''
class Test_Heading(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.imgProcessing.Heading import Heading
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.PointDetection import PointDetection
		self.Settings 		= Settings
		self.Heading 		= Heading
		self.PointDetection = PointDetection
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_Heading(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		###### START TEST #####
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				for fn_frame, fn_slframe in left_frames:
					self.TestHeading(folder, fn_frame, fn_slframe, draw=self.draw_frames)
		###########################

	def TestHeading(self, folder, fn_frame, fn_slframe, draw=True):
		'''
		 @brief Test function for Heading unit.

		 @param folder Input folder 
		 @param fn_frame Frame filename without points.
		 @param fn_slframe Frame filename with points.
		 @param draw (default=True - False will test speed and show only heading in frame)
		'''
		import timeit, warnings
		import numpy as np
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, MatplotShow
		from Settings.Exceptions import DroneVisionError

		print '\n'
		print '#----------- TESTING HEADING PROCESSING   \t---------------#'
		print '#----------- Image without points: {0} \t---------------#'.format(fn_frame)
		print '#----------- Image with points: {0}    \t---------------#'.format(fn_slframe)

		settings_inst = self.Settings.Settings()

		fn_frame 	= folder + fn_frame
		fn_slframe 	= folder + fn_slframe

		delay = timeit.default_timer()
		frame 		= GetImage(fn_frame)
		sl_frame 	= GetImage(fn_slframe)
		print 'Delay reading images: {0} sec'.format(timeit.default_timer() - delay)

		heading 	= self.Heading.Heading(settings_inst.GetSettings('CV', 'rho_step_distance'), settings_inst.GetSettings('CV', 'rho_min_diag_perc'))

		pointDet 	= self.PointDetection.PointDetection(True, settings_inst.GetSettings())
		pointDet.CalibratePointDetection()
		print 'Min distance between blobs: {0}'.format(pointDet.GetMinDistanceBetweenBlobs())

		total_delay = timeit.default_timer()

		delay = timeit.default_timer()
		try:
			delta_frame, point_kp, blob_desc, frame_un, sl_frame_un = pointDet.GetPointList(frame, sl_frame, draw=draw)
		except DroneVisionError, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
			return
		point_list_delay = timeit.default_timer() - delay

		delay = timeit.default_timer()
		try:
			hough_frame, edgel_map_filtered, boundary_hough_lines = pointDet.GetBoundaryHoughLines(frame_un, delta_frame, point_kp, draw=draw, print_hough_positions=draw)
		except DroneVisionError, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
			return
		hough_bound_delay = timeit.default_timer() - delay

		delay = timeit.default_timer()
		try:
			blade_heading, tip_or_root_heading, tip_or_root_detected, edgel_map_filtered = heading.ComputeHeading(edgel_map_filtered, boundary_hough_lines, draw_possible_edge_headings=draw, draw_headings=draw)
		except DroneVisionError, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
		heading_delay = timeit.default_timer() - delay

		timeout = timeit.default_timer() - total_delay
		print 'Total delay for finding heading: {0:.2f} sec, point list delay =  {1:.2f} sec, edge detection delay = {2:.2f} sec, heading processing delay = {3:.2f} sec'.format(timeout, point_list_delay, hough_bound_delay, heading_delay)

		edgel_map_filtered_all_headings = np.array(edgel_map_filtered, dtype=edgel_map_filtered.dtype)
		tip_or_root_detected = False
		hough_frame, edgel_map_filtered_all_headings, boundary_hough_lines = pointDet.GetBoundaryHoughLines(frame_un, delta_frame, point_kp, draw=True, print_hough_positions=True)
		try:
			blade_heading, tip_or_root_heading, tip_or_root_detected, edgel_map_filtered_all_headings = heading.ComputeHeading(edgel_map_filtered_all_headings, boundary_hough_lines, draw_possible_edge_headings=True, draw_headings=True)
		except DroneVisionError, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')

		touple_frames = []
		#touple_frames.append(('SL frame', sl_frame))
		touple_frames.append(('SL Frame', sl_frame_un))
		#touple_frames.append(('Original points', delta_frame))
		#touple_frames.append(('Hough lines', hough_frame))
		touple_frames.append(('Heading', edgel_map_filtered))
		#touple_frames.append(('Edge Headings', edgel_map_filtered_all_headings))
		print 'max_hor = DARK BLUE, min_hor = LIGHT BLUE, max_vert = PURPLE, min_vert = GREEN'
		if tip_or_root_detected:
			print 'heading = ORANGE, edge heading = YELLOW, tip/root heading = RED'
			print 'Tip or root detected: rho, theta = {0}'.format(tip_or_root_heading)
		else:
			print 'heading = ORANGE, edge heading = YELLOW'
			print 'Tip or root not detected'
		if not(self.CheckAllTests()):
			print 'PLOTTING FRAMES'
			MatplotShow(touple_frames, fn_frame+'_Heading', savefig_folder=self.savefig_folder+'heading_test/', save_fig=self.save_figs, save_fig_only=self.save_figs_only, inlude_main_title_in_plot=False)