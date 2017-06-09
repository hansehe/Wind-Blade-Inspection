'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''
import cv2, math, glob, os, warnings
import numpy as np
from Settings.Exceptions import DroneVisionError
from src.bin.SaveParameters import SaveParameters
from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape, CheckColor, PyrDown
from ..BlobDetector.BlobDetector import BlobDetector

'''
 @brief Class for computing standard distance between blobs based on scaling between structural light points.
 	A scale calibration sequence is initiated if it has not been previously done, or stereo_reset is set to True.
 	All scale frame calibration samples must be placed in the 'scale_calib_folder' folder, with following folder structure.
 	All structured light frames in a 'sl' folder and all normal frames in 'normal' folder.
 	The paired frames must have the same filenames, so that the BlobScaleDetector class may pair the frames for calibration.

 	Any number of frames may be used for calibration, 
 	The calibration samples should show an evenly distributed set of structured light points.

 @param me_master (True if this is the Master instance, False for slave instance)
 @param blob_scale_settings_inst (BLOB_SCALE settings)
 @param calib_settings_inst (CALIB settings)
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
class BlobScaleDetector(BlobDetector):
	def __init__(self, me_master, blob_scale_settings_inst, calib_settings_inst, default_downsampling_divisor, desired_frame_shape, reset,  detector_type=0, plot_figure=None):
		'''CONSTRUCTOR'''
		BlobDetector.__init__(self, me_master, calib_settings_inst, default_downsampling_divisor, desired_frame_shape, reset,  detector_type=detector_type, plot_figure=plot_figure)
		self.__saveParameters 				= SaveParameters(blob_scale_settings_inst.GetSettings('scale_calib_save_folder'), blob_scale_settings_inst.GetSettings('scale_calib_save_fname'), True)
		self.__calib_folder					= blob_scale_settings_inst.GetSettings('scale_calib_folder')
		self.__image_type 					= blob_scale_settings_inst.GetSettings('scale_img_type')
		self.__filtrate_points 				= blob_scale_settings_inst.GetSettings('scale_filtrate')
		self.__calib_reset 					= reset
		self.__scale_params 				= {}

		#Set by CalibrateStandardScaling
		self.__scale_params['standard_scale_distance'] = 100  #pixels

	def CalibrateBlobScaleDetector(self, printInfo=False, force_calibration=False, force_blob_calibration=False):
		'''
		 @brief Calibrate the blob scale detector

		 @param printInfo (Print info during calibration (default=False))
		 @param force_calibration (True/False for forcing new full calibration)
		 @param force_blob_calibration (True/False)
		'''
		self.CalibrateStereoVisionSystem(force_calibration=force_calibration, default_frame_shape=self.GetDesiredFrameShape())
		if not(self.LoadBlobScaleParameters()) or self.__calib_reset or force_calibration or force_blob_calibration:
			self.CalibrateStandardScaling(printInfo=printInfo)
			self.SaveBlobScaleParameters()
		self.CalibrateBlobDetector(self.GetStandardScaleDistance(), self.GetStandardPointSize())
		
	def CalibrateStandardScaling(self, printInfo=False):
		'''
		 @brief Run standard scale calibration.

		 @param printInfo (Print info during calibration (default=False))
		'''
		self.SetScalingImages()
		
		standard_scale_distances = []
		mean_point_sizes 		 = []
		for sl_fname, fname in self.__calib_img_fnames:
			frame 													= GetImage(fname, gray=False)
			sl_frame 												= GetImage(sl_fname, gray=False)
			try:
				delta_frame, keypoints, descriptors, frame, sl_frame = self.GetPointList(frame, sl_frame, concatenate_points=True, compute_descriptors=False)
			except DroneVisionError, err:
				warnings.simplefilter('always')
				warnings.warn(str(err) + ' - file: {0}, SL file: {1}'.format(fname, sl_fname), Warning)
				warnings.simplefilter('default')
				continue
			width, height 											= GetShape(sl_frame)
			blockSize 												= int(math.sqrt(math.pow(width, 2.0) + math.pow(height, 2.0))//2)
			scaling_point_frame, mean_point_size 					= self.PointScaleMatch(delta_frame, keypoints, blockSize)
			standard_scale_distance 								= self.ComputeStandardScaleDistance(scaling_point_frame)
			if np.isnan(standard_scale_distance):
				err = 'No scaling points detected - file: {0}, SL file: {1}'.format(fname, sl_fname)
				warnings.simplefilter('always')
				warnings.warn(str(err), Warning)
				warnings.simplefilter('default')
				continue

			standard_scale_distances.append(standard_scale_distance)
			mean_point_sizes.append(mean_point_size)
		
		if len(standard_scale_distances) == 0:
			raise ValueError('Could not compute any scaling points - please provide adequate frame samples!')
		self.__scale_params['standard_scale_distance'] 	= np.mean(standard_scale_distances)
		self.__scale_params['mean_point_size'] 			= np.mean(mean_point_sizes)
		if printInfo:
			print 'Standard blob scale calibration parameter: ', self.__scale_params['standard_scale_distance']

	def SetScalingImages(self):
		'''
		 @brief Get all scaling calibration image filenames
		'''
		self.__calib_img_fnames = []
		if not(os.path.isdir('./'+self.__calib_folder)):
			raise Exception('Scale calibration folder does not exist: {0}'.format(self.__calib_folder))
		calib_sl_img_fnames 		= glob.glob(self.__calib_folder+'sl/*.'+self.__image_type)
		calib_normal_img_fnames 	= glob.glob(self.__calib_folder+'normal/*.'+self.__image_type)
		if len(calib_sl_img_fnames) == 0 or len(calib_normal_img_fnames) == 0:
			raise Exception('No scale calibration images present in scale calibration folder: {0}'.format(self.__calib_folder))
		for sl_fname in calib_sl_img_fnames:
			fname = self.__saveParameters.MatchCalibFrames(sl_fname, calib_normal_img_fnames, self.__calib_folder+'sl/', self.__calib_folder+'normal/')
			self.__calib_img_fnames.append((sl_fname, fname))

	def GetScalingImages(self):
		'''
		 @brief Get all scaling calibration image filenames

		 @return calib_img_fnames
		'''
		self.SetScalingImages()
		return self.__calib_img_fnames

	def FiltrateScalingPoints(self, scaling_point_frame):
		'''
		 @brief Filtrate scalings by removing distances outside of the standard deviation.

		 @param scaling_point_frame

		 @return scaling_point_frame (filtrated)
		'''
		distance_pos_list = np.argwhere(scaling_point_frame > 0)
		if len(distance_pos_list) > 0:
			distances = np.zeros(len(distance_pos_list))
			for i in range(len(distance_pos_list)):
				x = distance_pos_list[i,0]
				y = distance_pos_list[i,1]
				distances[i] = scaling_point_frame[x,y]
			remove_ind = np.argwhere(np.abs(distances - np.mean(distances)) > np.std(distances)).T[0]
			for i in range(len(remove_ind)):
				x = distance_pos_list[remove_ind[i], 0]
				y = distance_pos_list[remove_ind[i], 1]
				scaling_point_frame[x,y] = 0

		return scaling_point_frame

	def ComputeStandardScaleDistance(self, scaling_point_frame):
		'''
		 @brief Compute standard scale distance between points.

		 @param scaling_point_frame (Computed from a frame with known distance)

		 @return standard_scale_distance
		'''
		standard_scale_distance = self.GetAverageScaling(scaling_point_frame)
		return standard_scale_distance

	def GetStandardScaleDistance(self):
		'''
		 @brief Get the standard scale distance between the blobs

		 @return standard scale distance
		'''
		return self.__scale_params['standard_scale_distance']

	def GetStandardPointSize(self):
		'''
		 @brief Get the standard size of points

		 @return standard scale distance
		'''
		return self.__scale_params['mean_point_size']

	def GetAverageScaling(self, scaling_point_frame):
		'''
		 @brief Compute average scaling distance of the scaling distances within the standard deviation.

		 @param scaling_point_frame

		 @return average_scale_distance (np.nan if no scaling points where detected)
		'''
		distance_points 		= np.argwhere(scaling_point_frame > 0)
		average_scale_distance 	= np.nan
		if len(distance_points) > 0:
			distances 		= np.zeros(len(distance_points))
			for i in range(len(distance_points)):
				distances[i] = scaling_point_frame[distance_points[i,0], distance_points[i,1]]
			valid_dist 		= np.argwhere(np.abs(distances-np.mean(distances)) <= np.std(distances)).T[0]
			std_distances 	= np.zeros(len(valid_dist))
			for i in range(len(valid_dist)):
				std_distances[i] = distances[valid_dist[i]]
			average_scale_distance = np.mean(std_distances)
		return average_scale_distance

	def PointScaleMatch(self, frame, keypoints, blockSize, filtrate_scaling_points=None):
		'''
		 @brief Compute scaling map between points.

		 @param frame
		 @param keypoints
		 @param blockSize (Should be about a quarter of the frame size if the points are distributed evenly on the frame)
		 @param filtrate_scaling_points (If None, then is is set by the class instance settings (default=None))

		 @return scaling_point_frame, mean_point_size
		'''
		# Class instance settings
		if filtrate_scaling_points == None:
			filtrate_scaling_points = self.__filtrate_points

		height, width			= GetShape(frame)
		points_frame 			= np.zeros((height, width), dtype=np.uint16)
		scaling_point_frame 	= np.zeros((height, width))
		float_points_list 		= [None]*(width*height)
		mean_point_size = 0.0
		for point in keypoints:
			x = int(round(point.pt[1]))
			y = int(round(point.pt[0]))
			float_points_list[x*width + y] = point
			mean_point_size += point.size
			points_frame[x, y] = point.size
		mean_point_size /= len(keypoints)
		blockSize_radi = blockSize//2
		points_pos = np.argwhere(points_frame > 0)
		for i in range(len(points_pos)):
			x 			 = points_pos[i, 0]
			y 			 = points_pos[i, 1]
			point 		 = float_points_list[x*width + y]
			if point == None:
				raise Exception('None point: {0}'.format(point))
			x_offset_neg = 0
			x_offset_pos = 0
			y_offset_neg = 0
			y_offset_pos = 0
			if x - blockSize_radi < 0:
				x_offset_neg = x - blockSize_radi
			elif x + blockSize_radi >= height:
				x_offset_pos = x + blockSize_radi - (height - 1)
			if y - blockSize_radi < 0:
				y_offset_neg = y - blockSize_radi
			elif y + blockSize_radi >= width:
				y_offset_pos = y + blockSize_radi - (width - 1)
			x_block_start		= x - blockSize_radi - x_offset_neg
			y_block_start		= y - blockSize_radi - y_offset_neg
			block 				= points_frame[x_block_start: (x+1) + blockSize_radi - x_offset_pos, y_block_start: (y+1) + blockSize_radi - y_offset_pos]
			x_block_center 		= len(block)//2
			y_block_center 		= len(block[x_block_center])//2
			block[x_block_center, y_block_center] = 0
			block_disparities 	= np.argwhere(block > 0)
			shortest_distance 	= -1
			for j in range(len(block_disparities)):
				x_block 	= x_block_start + block_disparities[j, 0]
				y_block 	= y_block_start + block_disparities[j, 1]
				block_point = float_points_list[x_block*width + y_block]
				if block_point == None:
					raise Exception('None block point: {0}'.format(block_point))
				distance 	= math.sqrt(math.pow(point.pt[0] - block_point.pt[0], 2) + math.pow(point.pt[1] - block_point.pt[1], 2))
				if distance < shortest_distance or shortest_distance < 0:
					shortest_distance 	= distance
			if shortest_distance >= 0:
				scaling_point_frame[x,y] = shortest_distance

		if filtrate_scaling_points:
			scaling_point_frame = self.FiltrateScalingPoints(scaling_point_frame)

		return scaling_point_frame, mean_point_size

	def SaveBlobScaleParameters(self):
		'''
		 @brief Save blob scale parameters for later use.
		'''
		self.__saveParameters.Save(self.__scale_params)

	def LoadBlobScaleParameters(self):
		'''
		 @brief Load blob scale parameters

		 @return True/False - stereo parameters loaded successfully.
		'''
		ok, self.__scale_params = self.__saveParameters.Load()
		return ok