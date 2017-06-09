'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''
import cv2, math, glob, os, warnings
import numpy as np
from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape, CheckColor, CheckGrayScale, GetRandomColor
from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.BlobScaleDetector.BlobScaleDetector import BlobScaleDetector
from Settings.Exceptions import DroneVisionError

'''
 @brief Class for feature based stereopsis.
 	A stereopsis calibration sequence is initiated if it has not been previously done, or stereo_reset is set to True.
 	All stereopsis frame calibration samples must be placed in the 'stereopsis_calib_folder' folder, with following folder structure.
 	All left frames in a 'left' folder, with all structured light frames in a 'sl' subfolder and all normal frames in 'normal' subfolder.
 	All right frames in a 'right' folder, with all structured light frames in a 'sl' subfolder and all normal frames in 'normal' subfolder.
 	The paired frames must have the same filenames, so that the FeatureStereopsis class may pair the frames for calibration.
 	Each filename group must be as follows: '<baseline in mm>mm.<steropsis_calib_img_type>'
 		For example, a frame group with 10mm baseline must all have filename as '10mm.jpeg' (if it's a jpeg file).
 		All baselines are measured in mm, and the FeatureStereopsis class finds the baseline for each frame group in the filename.
 		The focal length is considered fixed according to the settings.

 	Any number of frames may be used for calibration, 
 	The calibration samples should show an evenly distributed set of structured light points.

 @param me_master (True if this is the Master instance, False for slave instance)
 @param feature_settings_inst (F_STEREO settings)
 @param calib_settings_inst (CALIB settings)
 @param blob_scale_settings_inst (BLOB_SCALE settings)
 @param default_downsampling_divisor (Default divisor for downsampling frames)
 @param desired_frame_shape (desired frame shape of processed frames. Incoming frames are matched to fit this shape. Given as a tuple of (height, width). Set (-1,-1) to fix downscaling to the default_downsampling_divisor.)
 @param reset (True/False)
 @param detector_type (	flag for which detector to use: 
		 						0 = Simple blob detector (default)
		 						1 = ORB detector
		 						2 = SIFT detector
		 						3 = SURF detector)
 @param plot_figure (optional plot figure (default=None))
'''
class FeatureStereopsis(BlobScaleDetector):
	def __init__(self, me_master, feature_stereo_settings_inst, calib_settings_inst, blob_scale_settings_inst, default_downsampling_divisor, desired_frame_shape, reset, detector_type=0, plot_figure=None):
		'''CONSTRUCTOR'''
		BlobScaleDetector.__init__(self, me_master, blob_scale_settings_inst, calib_settings_inst, default_downsampling_divisor, desired_frame_shape, reset, detector_type=detector_type, plot_figure=plot_figure)
		self.__use_triangulation			= feature_stereo_settings_inst.GetSettings('use_triangulation')
		self.__use_cv2_triangulation 		= feature_stereo_settings_inst.GetSettings('use_cv2_triangulation')
		self.__use_block_matching 			= feature_stereo_settings_inst.GetSettings('use_block_matching')
		self.__block_matching_param			= feature_stereo_settings_inst.GetSettings('block_matching_parameter')
		self.__use_brute_force_matching 	= feature_stereo_settings_inst.GetSettings('use_brute_force')

	def CalibrateFeatureStereopsis(self, printInfo=False, force_calibration=False, force_blob_calibration=False):
		'''
		 @brief Run standard stereopsis calibration.

		 @param printInfo (Print info during calibration (default=False))
		 @param force_calibration (True/False for forcing new full calibration)
		 @param force_blob_calibration (True/False)
		'''
		self.CalibrateBlobScaleDetector(printInfo=printInfo, force_calibration=force_calibration, force_blob_calibration=force_blob_calibration)

	def Get3DPoints(self, left_delta_frame_shape, right_delta_frame_shape, keypoints_l, descriptors_l, keypoints_r, descriptors_r, filtrate_3Dpoints=True, concatenate_points=False, draw=False, left_delta_frame=None, right_delta_frame=None):
		'''
		 @brief Steps for computing stereopsis (3D points) based on translated frames (left frame and right frame).

		 @param left_delta_frame_shape (Left frame with highlighted points above threshold)
		 @param right_delta_frame_shape (tuple as (height, width) of right frame)  
		 @param keypoints_l (all point positions (2D numpy array = each row is a point position [x, y]) from left frame)
		 @param descriptors_l (all point descriptions from left frame)
		 @param keypoints_r (all point positions (2D numpy array = each row is a point position [x, y]) from right frame)
		 @param descriptors_r (all point descriptions from right frame)
		 @param filtrate_3Dpoints (Filtrate 3D points by removing all 3D points with depth outside of the standard deviation of all depths (default=True).)
		 @param concatenate_points (True/False on concatenating close points before processing stereopsis (default=False))
		 @param draw (default=False)
		 @param left_delta_frame (Input for left frame if it is available, None if not (default=None))
		 @param right_delta_frame (Input for right_delta_frame if it is available, None if not (default=None))

		 if draw:
		 	@return points3D, match_points_frame
		 else:
		 	@return points3D
		'''
		if concatenate_points:
			print 'CONCATENETING??'
			keypoints_l, descriptors_l = self.ConcatenateClosePoints(keypoints_l, descriptors_l)
			keypoints_r, descriptors_r = self.ConcatenateClosePoints(keypoints_r, descriptors_r)

		matches = self.ComputePointMatches(left_delta_frame_shape, right_delta_frame_shape, keypoints_l, descriptors_l, keypoints_r, descriptors_r)
		if len(matches) == 0:
			raise DroneVisionError('no_3D_point_matches')

		if draw: 
			if not(isinstance(left_delta_frame, np.ndarray)):
				left_delta_frame = self.CreateKeypointsFrame(left_delta_frame_shape, keypoints_l)
			if not(isinstance(right_delta_frame, np.ndarray)):
				right_delta_frame = self.CreateKeypointsFrame(right_delta_frame_shape, keypoints_r)
			match_points_frame = self.DrawMatches(left_delta_frame, right_delta_frame, keypoints_l, keypoints_r, matches)

		points3D = self.Compute3DPoints(keypoints_l, keypoints_r, matches)
		if filtrate_3Dpoints:
			try:
				points3D = self.FiltratePoints3D(points3D)
			except Exception, err:
				warnings.simplefilter('always')
				warnings.warn(str(err), Warning)
				warnings.simplefilter('default')
				raise DroneVisionError('3D_point_filtration_failed')

		if draw:
			return points3D, match_points_frame
		return points3D

	def GetUsingTriangulation(self):
		'''
		 @brief Get the 3D coordinate computation method in use.

		 @return True/False (True for using triangulation, False for using simple f*T/d computation)
		'''
		return self.__use_triangulation

	def GetUsingBlockMatching(self):
		'''
		 @brief Get the matching method. True for using block matching method or False for using FLANN/BRUTE force method.

		 @return True/False
		'''
		return self.__use_block_matching

	def ComputePointMatches(self, left_delta_frame_shape, right_delta_frame_shape, left_keypoints, left_descriptors, right_keypoints, right_descriptors):
		'''
		 @brief Compute matching feature points by using block matching (between the feature points) or the FLANN/Brute force method.

		 @param left_delta_frame_shape
		 @param right_delta_frame_shape
		 @param left_keypoints
		 @param left_descriptors
		 @param right_keypoints
		 @param right_descriptors

		 @return matches
		'''
		if self.GetUsingBlockMatching():
			matches = self.PointBlockMatch(left_delta_frame_shape, right_delta_frame_shape, left_keypoints, right_keypoints, minBlobDistanceScaleParameter=self.__block_matching_param)
		else:
			matches = self.PointFLANNMatch(left_keypoints, left_descriptors, right_keypoints, right_descriptors, use_brute_force_matching=self.__use_brute_force_matching)
		return matches

	def Compute3DPoints(self, left_keypoints, right_keypoints, matches):
		'''
		 @brief Compute 3D points using trangulation or f*T/d method.

		 @param left_keypoints
		 @param right_keypoints
		 @param matches

		 @return points3D
		'''
		if self.GetUsingTriangulation():
			points3D = self.TriangulatePoints(left_keypoints, right_keypoints, matches, opencv_triangulation=self.__use_cv2_triangulation)
		else:
			points3D = self.Compute3DPointsFromDisparity(left_keypoints, right_keypoints, matches)
		return points3D

	def GetBaselineFromFilename(self, folder, fname):
		'''
		 @brief Get baseline in mm from filename, when the filename is of '<baseline in mm>mm.<steropsis_calib_img_type>'

		 @param folder (folder name containing the file)
		 @param fname (filename)

		 @return baseline (float)
		'''
		sh_fname = fname[len(folder):]
		baseline = float(sh_fname[:-(3+len(self.__image_type))]) #removing mm.<steropsis_calib_img_type>
		return baseline

	def GetStereopsisImages(self):
		'''
		 @brief Get all stereopsis calibration image filenames

		 @return calib_img_fnames
		'''
		self.SetStereopsisImages()
		return self.__calib_img_fnames

	def PointFLANNMatch(self, left_keypoints, left_descriptors, right_keypoints, right_descriptors, use_brute_force_matching=False):
		'''
		 @brief Compute matches between the left and right points using the BFMatcher() (brute force) or the FlannBasedMatcher().

		 @param left_keypoints
		 @param left_descriptors
		 @param right_keypoints
		 @param right_descriptors
		 @param use_brute_force_matching (True for using brute force based matcher, false for using FLANN based matcher (default=False))
		 @param draw

		 @return disparity_point_frame (float)
		'''
		if use_brute_force_matching:
			matcher = cv2.BFMatcher(cv2.NORM_L1, crossCheck=True)
			matches = matcher.match(right_descriptors, left_descriptors)
		else:
			# FLANN parameters
			FLANN_INDEX_KDTREE = 0
			index_params 	= dict(algorithm = FLANN_INDEX_KDTREE, trees = 1)
			search_params 	= dict(checks=50)   # or pass empty dictionary
			matcher 		= cv2.FlannBasedMatcher(index_params,search_params)
			flann_matches 	= matcher.knnMatch(right_descriptors, left_descriptors, k=7)
			#flann_matches 	= matcher.radiusMatch(right_descriptors, left_descriptors, maxDistance=500)
			# ratio test as per Lowe's paper
			matches = []
			for i, knn_matches in enumerate(flann_matches):
				if len(knn_matches) > 1:
					m = knn_matches[0]
					n_distance = 0.0
					for j in range(1, len(knn_matches)):
						n_distance += knn_matches[j].distance
					n_distance /= len(knn_matches)-1
					if m.distance < 0.7*n_distance:
						matches.append(m)
				else:
					matches.append(knn_matches[0])
		return matches

	def PointBlockMatch(self, left_frame_shape, right_frame_shape, left_keypoints, right_keypoints, minBlobDistanceScaleParameter=2.5, blockSize=None):
		'''
		 @brief Compute disparities between matching points.

		 @param left_frame_shape (tuple of (height,width))
		 @param right_frame_shape (tuple of (height,width))
		 @param left_keypoints
		 @param right_keypoints
		 @param minBlobDistanceScaleParameter (scaler for increasing/decreasing search radius for matching points.)
		 @param blockSize (odd > 1 and positive, If None, then scale blob distance from pointDetection instance is used (default=None).)

		 @return matches (list of cv2.DMatch instances)
		'''
		if blockSize == None:
			self.AssertBlobDistanceCalibrated()
			blockSize = int(round(self.GetMinDistanceBetweenBlobs()*minBlobDistanceScaleParameter))
			if blockSize % 2 == 0: # Asserting that blockSize is odd.
				blockSize += 1

		left_points_frame 		= np.zeros(left_frame_shape, dtype=np.uint16)
		right_points_frame 		= np.zeros(right_frame_shape, dtype=np.uint16)
		left_index_points_list 	= np.ones(left_frame_shape[0]*left_frame_shape[1], dtype=int)*-1
		right_index_points_list = np.ones(right_frame_shape[0]*right_frame_shape[1], dtype=int)*-1
		mean_blob_size 			= 0.0

		i, j = 0, 0
		length_left_keypoints 	= len(left_keypoints)
		length_right_keypoints 	= len(right_keypoints)
		while i < length_left_keypoints or j < length_right_keypoints:
			if i < length_left_keypoints:
				x = int(round(left_keypoints[i].pt[1]))
				y = int(round(left_keypoints[i].pt[0]))
				left_index_points_list[x*left_frame_shape[1] + y] = i
				left_points_frame[x, y] = left_keypoints[i].size
				mean_blob_size += left_keypoints[i].size
				i += 1
			if j < length_right_keypoints:
				x = int(round(right_keypoints[j].pt[1]))
				y = int(round(right_keypoints[j].pt[0]))
				right_index_points_list[x*right_frame_shape[1] + y] = j
				right_points_frame[x, y] = right_keypoints[j].size
				mean_blob_size += right_keypoints[j].size
				j += 1

		mean_blob_size 	/= len(left_keypoints) + len(right_keypoints)

		blob_size_error_thres 	= mean_blob_size/4
		blockSize_radi_y 		= blockSize//2 		# Sorry but the (x,y) coordinates is according to normal matrix axes (x=row, y=columns), not image coordinates (y=rows, x=columns).
		blockSize_radi_x 		= blockSize//6

		matches = []
		left_points_pos = np.argwhere(left_points_frame > 0)
		for i in range(len(left_points_pos)):
			x = left_points_pos[i, 0]
			y = left_points_pos[i, 1]
			x_offset_neg = 0
			x_offset_pos = 0
			y_offset_neg = 0
			if x - blockSize_radi_x < 0:
				x_offset_neg = x - blockSize_radi_x
			elif x + blockSize_radi_x >= left_frame_shape[0]:
				x_offset_pos = x + blockSize_radi_x - (left_frame_shape[0] - 1)
			if y - blockSize_radi_y < 0:
				y_offset_neg = y - blockSize_radi_y
			x_block_start			= x - blockSize_radi_x - x_offset_neg
			y_block_start			= y - blockSize_radi_y - y_offset_neg
			right_block 			= right_points_frame[x_block_start: (x+1) + blockSize_radi_x - x_offset_pos, y_block_start: (y+1)]
			right_block_disparities = np.argwhere(right_block > 0)
			shortest_distance 		= -1
			left_point 				= left_keypoints[left_index_points_list[x*left_frame_shape[1] + y]]
			for j in range(len(right_block_disparities)):
				x_right 	= x_block_start + right_block_disparities[j, 0]
				y_right 	= y_block_start + right_block_disparities[j, 1]
				right_point = right_keypoints[right_index_points_list[x_right*right_frame_shape[1] + y_right]]
				distance 	= math.sqrt(math.pow(left_point.pt[0] - right_point.pt[0], 2) + math.pow(left_point.pt[1] - right_point.pt[1], 2))
				size_error 	= np.abs(left_point.size - right_point.size)
				if (distance < shortest_distance or shortest_distance < 0) and size_error < blob_size_error_thres:
					shortest_distance 	= distance
					shortest_x_right 	= x_right
					shortest_y_right 	= y_right
			if shortest_distance >= 0:
				match = cv2.DMatch()
				match.trainIdx 	= left_index_points_list[x*left_frame_shape[1] + y]
				match.queryIdx 	= right_index_points_list[shortest_x_right*right_frame_shape[1] + shortest_y_right]
				matches.append(match)

		return matches

	def DrawMatches(self, frame_l, frame_r, left_keypoints, right_keypoints, matches):
		'''
		 @brief Own implementation of drawMatches between two frames

		 @param frame_l
		 @param frame_r
		 @param left_keypoints
		 @param right_keypoints
		 @param matches (list of cv2.DMatch instances)

		 @return match_frame
		'''
		height_l, width_l 	= GetShape(frame_l)
		height_r, width_r	= GetShape(frame_r)
		if height_l > height_r:
			height = height_l
		else:
			height = height_r
		match_frame 		= np.zeros((height,width_l+width_r), dtype=frame_l.dtype)
		match_frame[:height_l, :width_l] = CheckGrayScale(frame_l)
		match_frame[:height_r, width_l:] = CheckGrayScale(frame_r)
		match_frame 		= CheckColor(match_frame)
		left_pts, right_pts = self.GetMatchingPoints(left_keypoints, right_keypoints, matches)
		for i in range(len(left_pts)):
			cv2.line(match_frame, (int(round(left_pts[i].pt[0])), int(round(left_pts[i].pt[1]))), (width_l+int(round(right_pts[i].pt[0])), int(round(right_pts[i].pt[1]))), GetRandomColor(), 1, lineType=cv2.LINE_AA)
		return match_frame

	def CreateKeypointsFrame(self, frame_shape, keypoints, size_delimiter_param=2.0):
		'''
		 @brief Create frame with keypoints given by size and position of the keypoints.

		 @param frame_shape (Use GetShape(frame) of a grayscale frame)
		 @param keypoints
		 @param size_delimiter_param (Divisor for tuning the size of drawn keypointn blobs (default=2.0))

		 @return keypoints_frame
		'''
		keypoints_frame = np.zeros(frame_shape, dtype=np.uint8)
		for i in range(len(keypoints)):
			y 		= int(round(keypoints[i].pt[1]))
			x 		= int(round(keypoints[i].pt[0]))
			size 	= int(round(keypoints[i].size/size_delimiter_param))
			cv2.circle(keypoints_frame, (x, y), size, 255, -1)
		return keypoints_frame