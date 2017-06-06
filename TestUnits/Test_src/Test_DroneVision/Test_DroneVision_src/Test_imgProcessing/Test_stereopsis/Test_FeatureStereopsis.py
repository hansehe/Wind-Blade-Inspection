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
 @brief Test unit for FeatureStereopsis
'''
class Test_FeatureStereopsis(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.imgProcessing.stereopsis import FeatureStereopsis
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.PointDetection import PointDetection
		self.Settings 			= Settings
		self.FeatureStereopsis 	= FeatureStereopsis
		self.PointDetection 	= PointDetection
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_FeatureStereopsis(self):
		'''
		 @brief Main start test function.
		 	Append functions to test for this unit.
		'''
		###### START TEST #####
		from src.DroneVision.DroneVision_src.hardware.imageTools import MatplotShow
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				self.first_reset = self.reset_calibration
				for i in range(len(left_frames)):
					match_frames = []
					match_frames.append(self.TestPointMatchStereoVision(folder, left_frames[i][0], left_frames[i][1], right_frames[i][0], right_frames[i][1], baselines[i], actual_distances[i], filtrate_3Dpoints=self.filtrate_3Dpoints, use_triangulation=self.use_triangulation, use_opencv_triangulation=self.use_opencv_triangulation, use_block_matching=True)) # Block matching
					self.first_reset = False
					if self.test_FLANN_matching:
						match_frames.append(self.TestPointMatchStereoVision(folder, left_frames[i][0], left_frames[i][1], right_frames[i][0], right_frames[i][1], baselines[i], actual_distances[i], filtrate_3Dpoints=self.filtrate_3Dpoints, use_triangulation=self.use_triangulation, use_opencv_triangulation=self.use_opencv_triangulation, use_block_matching=False, use_brute_force_matching=False)) # FLANN based matcher
					if self.test_BRUTE_FORCE_matching:
						match_frames.append(self.TestPointMatchStereoVision(folder, left_frames[i][0], left_frames[i][1], right_frames[i][0], right_frames[i][1], baselines[i], actual_distances[i], filtrate_3Dpoints=self.filtrate_3Dpoints, use_triangulation=self.use_triangulation, use_opencv_triangulation=self.use_opencv_triangulation, use_block_matching=False, use_brute_force_matching=True)) # Brute Force based matcher
					if not(self.CheckAllTests()):
						MatplotShow(match_frames, left_frames[i][0]+'_Feature_stereo_test', save_fig=self.save_figs, save_fig_only=self.save_figs_only)
		###########################

	def TestPointMatchStereoVision(self, folder, left_fn_frame, left_fn_slframe, right_fn_frame, right_fn_slframe, baseline, actual_distance, filtrate_3Dpoints=True, use_triangulation=True, use_opencv_triangulation=True, use_block_matching=True, use_brute_force_matching=False):
		'''
		 @brief Test function for stereo detection unit using point matching.

		 @param folder Input folder 
		 @param left_fn_frame Frame filename without points.
		 @param left_fn_slframe Frame filename with points.
		 @param right_fn_frame Frame filename without points.
		 @param right_fn_slframe Frame filename with points.
		 @param baseline (mm)
		 @param actual_distance (mm)
		 @param filtrate_3Dpoints (Filtrate 3D points which are outside of the standard deviation)
		 @param use_triangulation (True for using tringulation method, False for using easy disparity to depth method)
		 @param use_opencv_triangulation (True for using the opencv implementation of triangulation)
		 @param use_block_matching (use the block matching feature matching method)
		 @param use_brute_force_matching (use the brute force matching method (if use_block_matching=False))
		'''
		import timeit
		import numpy as np
		from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, MatplotShow

		if use_block_matching:
			title = 'BLOCK SEARCH'
		else:
			if use_brute_force_matching:
				title = 'BRUTE F.'
			else:
				title = 'FLANN'
		if use_triangulation:
			extra_info = 'TRIANGULATION TO DEPTH'
		else:
			extra_info = 'DISPARITY TO DEPTH'
		print '\n'
		print '#----------- TESTING FEATURE BASED STEREO DETECTION - {0} MATCHING - {1}\t---------------#'.format(title, extra_info)
		print '#----------- Left Image without points: {0} \t\t\t\t\t---------------#'.format(left_fn_frame)
		print '#----------- Right Image without points: {0} \t\t\t\t---------------#'.format(right_fn_frame)

		settings_inst = self.Settings.Settings()
		settings_inst.ChangeSetting('BASIC', 'reset_calibration', False)
		#settings_inst.ChangeSetting('CALIB', 'baseline', baseline)
		settings_inst.ChangeSetting('F_STEREO', 'use_triangulation', use_triangulation)
		settings_inst.ChangeSetting('F_STEREO', 'use_cv2_triangulation', use_opencv_triangulation)
		settings_inst.ChangeSetting('F_STEREO', 'use_block_matching', use_block_matching)
		settings_inst.ChangeSetting('F_STEREO', 'use_brute_force', use_brute_force_matching)
		left_pointDet 	= self.PointDetection.PointDetection(True, settings_inst.GetSettings())
		left_pointDet.CalibratePointDetection(printInfo=True)
		right_pointDet 	= self.PointDetection.PointDetection(False, settings_inst.GetSettings())
		right_pointDet.CalibratePointDetection(printInfo=False)

		#if not(left_pointDet.GetBaseline() == baseline) or not(right_pointDet.GetBaseline() == baseline):
		#	raise Exception('Fail baseline!')

		left_fn_frame 		= folder + left_fn_frame
		left_fn_slframe 	= folder + left_fn_slframe
		right_fn_frame 		= folder + right_fn_frame
		right_fn_slframe 	= folder + right_fn_slframe

		delay = timeit.default_timer()
		left_frame 		= GetImage(left_fn_frame)
		left_sl_frame 	= GetImage(left_fn_slframe)
		right_frame 	= GetImage(right_fn_frame)
		right_sl_frame 	= GetImage(right_fn_slframe)
		print 'Delay reading images: {0} sec'.format(timeit.default_timer() - delay)

		#MatplotShow([('left', left_sl_frame), ('right', right_sl_frame)], 'Feature Stereopsis', save_fig=self.save_figs, save_fig_only=self.save_figs_only)

		left_delta_frame, left_points_kp, left_blob_desc, left_frame, left_sl_frame 		= left_pointDet.GetPointList(left_frame, left_sl_frame, compute_descriptors=not(use_block_matching), draw=True)
		right_delta_frame, right_points_kp, right_blob_desc, right_frame, right_sl_frame 	= right_pointDet.GetPointList(right_frame, right_sl_frame, compute_descriptors=not(use_block_matching), draw=True)

		total_delay = timeit.default_timer()

		points3D, match_points_frame 	= left_pointDet.Get3DPoints(GetShape(left_delta_frame), GetShape(right_delta_frame), left_points_kp, left_blob_desc, right_points_kp, right_blob_desc, filtrate_3Dpoints=filtrate_3Dpoints, draw=True, left_delta_frame=left_delta_frame, right_delta_frame=right_delta_frame)
		points3D_m 						= left_pointDet.Points3DToMatrix(points3D)
		average_point3D 				= np.mean(points3D_m, axis=1)
		std_point3D 					= np.std(points3D_m, axis=1)

		timeout = timeit.default_timer() - total_delay
		print 'Total delay for feature based stereopsis: {0} sec'.format(timeout)

		delay = timeit.default_timer()
		points3D = left_pointDet.Get3DPoints(GetShape(left_delta_frame), GetShape(right_delta_frame), left_points_kp, left_blob_desc, right_points_kp, right_blob_desc, filtrate_3Dpoints=filtrate_3Dpoints, draw=False, left_delta_frame=left_delta_frame, right_delta_frame=right_delta_frame)
		timeout_stereopsis = timeit.default_timer() - delay

		for point3D in points3D:
			print 'Point3D: x = {0} \t y = {1} \t z = {2}'.format(point3D[0,0], point3D[1,0], point3D[2,0])
		print 'Average Point3D: x = {0} \t y = {1} \t z = {2}'.format(average_point3D[0,0], average_point3D[1,0], average_point3D[2,0])
		print 'STD Point3D: x = {0} \t y = {1} \t z = {2}'.format(std_point3D[0,0], std_point3D[1,0], std_point3D[2,0])
		print 'Delay for computing distance points: {0} sec, average distance: {1} mm, actual distance: {2} mm, distance error: {3} mm, baseline = {4} mm'.format(timeout_stereopsis, average_point3D[2,0], actual_distance, average_point3D[2,0] - actual_distance, baseline)

		#MatplotShow([(title, match_points_frame)], save_fig=self.save_figs, save_fig_only=self.save_figs_only)

		#title += ':Z(mean)={0:.2f}mm'.format(average_point3D[2,0])

		if not(self.CheckAllTests()):
			if self.show_delta_frames:
				MatplotShow([('left', left_delta_frame), ('right', right_delta_frame)], left_fn_frame+'_Feature_stereo_test_delta_frames', save_fig=self.save_figs, save_fig_only=self.save_figs_only)

		return (title, match_points_frame)