'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''
import math, operator
import numpy as np

from src.DroneVision.DroneVision_src.imgProcessing.stereopsis.FeatureStereopsis import FeatureStereopsis
from ..generalDetectors.detectLines import HoughLinesPointMatrix, FindLineLimits
from ..generalDetectors.detectEdges import DetectBoundaryEdges

'''
 @brief Class for point detection algorithms
 		Run CalibratePointDetection shortly after creating this class instance for better performance.

 @param me_master (True if this instance is master, False for slave)
 @param settings_inst (all settings)
 @param plot_figure (optional plot figure (default=None))
'''
class PointDetection(FeatureStereopsis):
	def __init__(self, me_master, settings_inst, plot_figure=None):
		'''CONSTRUCTOR'''
		FeatureStereopsis.__init__(self, me_master, settings_inst.GetSettings('F_STEREO'), \
			settings_inst.GetSettings('CALIB'), settings_inst.GetSettings('BLOB_SCALE'), \
			settings_inst.GetSettings('CV', 'default_downsampling_divisor'), \
			settings_inst.GetSettings('CV', 'desired_frame_shape'), \
			settings_inst.GetSettings('BASIC', 'reset_calibration'), \
			settings_inst.GetSettings('CV', 'detector_type'), \
			plot_figure=plot_figure)

	def CalibratePointDetection(self, printInfo=False, force_calibration=False):
		'''
		 @brief Calibrate BlobDetector, FeatureStereopsis and StereoCalibration.

		 @param printInfo (Print info during calibration (default=False))
		 @param force_calibration (True/False for forcing new full calibration)
		'''
		self.CalibrateFeatureStereopsis(printInfo=printInfo, force_calibration=force_calibration)

	def GetBoundaryHoughLines(self, origin_frame, delta_frame, keypoints, filtrate_edge_points=True, use_min_blob_distance_threshold=True, draw=False, print_hough_positions=False):
		'''
		 @brief Steps for computing boundary hough lines from a point list.

		 @param origin_frame (Original grayscale frame without laser points)
		 @param delta_frame (frame with highlighted points above threshold)
		 @param keypoints (all point positions (list of keypoints))
		 @param filtrate_edge_points (Default=True)
		 @param use_min_blob_distance_threshold (Use precomputed minimum blob distance as distance thresholding (default=True))
		 @param draw (default=False)
		 @param print_hough_positions (print hough line positions (rho, theta) (default=False))

		 @return hough_frame, edgel_map, boundary_hough_lines, edge_points (see return description on DetectBoundaryEdges and FindLineLimits)
		'''
		if draw:
			hough_frame = np.array(delta_frame, dtype=delta_frame.dtype)
		else:
			hough_frame = delta_frame #Copy pointer only

		if self.GetBlobDistanceCalibrated() and use_min_blob_distance_threshold:
			radi_threshold = self.GetMinDistanceBetweenBlobs()
		else:
			radi_threshold = None
			
		hough_lines 									= HoughLinesPointMatrix(delta_frame, keypoints, radi_threshold=radi_threshold, radi_threshold_tuning_param=0.3)
		hough_frame, bounded_lines, max_min_lines 		= FindLineLimits(hough_frame, hough_lines, keypoints, radi_threshold=radi_threshold, radi_threshold_tuning_param=0.5, draw_hough_matrix=draw, draw_bounded_lines=draw, draw_arrowed_bounded_lines=True)
		edgel_map, boundary_hough_lines 				= DetectBoundaryEdges(origin_frame, bounded_lines, max_min_lines, filtrate_edge_points=filtrate_edge_points, draw=draw, print_hough_positions=print_hough_positions)

		return hough_frame, edgel_map, boundary_hough_lines