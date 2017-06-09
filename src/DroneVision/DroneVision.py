'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

'''
 Import libraries
'''
import warnings
import numpy as np

# Settings
from Settings.Settings import Settings
from Settings.Exceptions import DroneVisionError

# Extra tools
from src.bin.tools import CheckDir

# Hardware Link
from DroneVision_src.hardware.ImageLink import ImageLink
from DroneVision_src.hardware.CameraLink import CameraLink
from DroneVision_src.hardware.VideoLink import VideoLink
from DroneVision_src.hardware.imageTools import ImShow

# Camera calibration
from DroneVision_src.imgProcessing.featureDetection.PointDetection.PointDetection import PointDetection

# Heading calculation
from DroneVision_src.imgProcessing.Heading.Heading import Heading

'''
 @brief DroneVision class. Handle cameras and computer vision.

 @param settings_inst - For preconfigured settings. None if no preconfigured settings are available. (default=None)
 @param me_master True/False - True = this is master (left camera), False = this is slave (right camera) (default=True)
 @param plot_figure (optional plot figure (default=None))
'''
class DroneVision(PointDetection, Heading, Settings):
	def __init__(self, me_master, settings_inst=None, plot_figure=None):
		'''CONSTRUCTOR'''
		Settings.__init__(self, load_initial_settings=False)
		if not(isinstance(settings_inst, type(None))):
			self.ResetSettings(settings_inst.GetRawSettings())
		else:
			self.GetInitialSettings()
		self.__me_master 				= me_master
		self.__droneVision_ready 		= False
		self.__crop_frames 				= False
		self.__delta_fan_angle_divisor 	= 1.0
		if (self.GetSettings('LASER', 'fan_angle') < self.GetSettings('CAMERA', 'fan_angle')) and self.GetSettings('CV', 'crop_frames'):
			self.__crop_frames 				= True
			self.__delta_fan_angle_divisor 	= self.GetSettings('LASER', 'fan_angle') / self.GetSettings('CAMERA', 'fan_angle')
		self.CheckManualTriggeringAndAutoMode()
		PointDetection.__init__(self, self.__me_master, self.GetSettings(), plot_figure=plot_figure)
		Heading.__init__(self, self.GetSettings('CV', 'rho_step_distance'), self.GetSettings('CV', 'rho_min_diag_perc'))

	################## INIT FUNCTIONS ########################

	def GetDroneVisionReady(self):
		'''
		 @brief Get bool to check if droneVision is intialized and ready.

		 @return True/False
		'''
		return self.__droneVision_ready

	def InitDroneVision(self):
		'''
			Init drone vision class.
				- Turn on cameras
				- Create instances of video recorders 
					- Both master and slave records original frames.
					- Disparity map (result) are only recorded by master
				- Create instance of heading class (master only)
		'''
		self.CreateFolders()
		self.CreateCameraLink()
		self.__droneVision_ready = True

	def CreateFolders(self):
		'''
		 @brief Create folders before creating camera link or video recorders.
		'''
		# Create output_folder
		if self.__me_master:
			side_folder = 'left_camera/'
		else:
			side_folder = 'right_camera/'
		self.__output_folder = self.GetSettings('DATABASE', 'output_folder') + side_folder
		CheckDir(self.__output_folder)
			
	def CreateCameraLink(self):
		'''
		 @brief Create camera link (or video/image link)
		'''
		#---- CREATE IMAGE/VIDEO/CAMERA LINK ----#
		if self.GetSettings('BASIC', 'source_type') == 'CAMERA':
			self.objCameraLink = CameraLink(self.__me_master, self.GetSettings('CAMERA'), self.GetSettings('LASER'))
		elif self.GetSettings('BASIC', 'source_type') == 'VIDEO':
			input_folder = self.GetSettings('VIDEO', 'input_folder')
			if self.__me_master: # This is master
				filename = self.GetSettings('VIDEO', 'left_video')
				sl_filename = self.GetSettings('VIDEO', 'left_sl_video')
			else: # This is slave
				filename = self.GetSettings('VIDEO', 'right_video')
				sl_filename = self.GetSettings('VIDEO', 'right_sl_video')
			self.objCameraLink = VideoLink(input_folder + filename, input_folder + sl_filename)
		elif self.GetSettings('BASIC', 'source_type') == 'IMAGE':
			input_folder = self.GetSettings('IMAGE', 'input_folder')
			if self.__me_master: # This is master
				filename 	= self.GetSettings('IMAGE', 'left_images')
				sl_filename = self.GetSettings('IMAGE', 'left_sl_images')
			else: # This is slave
				filename = self.GetSettings('IMAGE', 'right_images')
				sl_filename = self.GetSettings('IMAGE', 'right_sl_images')
			self.objCameraLink = ImageLink(input_folder, filename, sl_filename)
		else:
			raise Exception('Invalid setting: ' + self.GetSettings('BASIC', 'source_type'))

	def CheckManualTriggeringAndAutoMode(self):
		'''
		 @brief Auto mode cannot be on if manual triggering is on.
		'''
		if self.GetSettings('BASIC', 'source_type') == 'CAMERA':
			if self.GetSettings('CAMERA', 'manual_triggering'):
				self.ChangeSetting('USER_INPUT', 'automatic_mode', False)

	#############################################################

	def GetCameraOutputFolder(self):
		'''
		 @brief Get output folder for the camera

		 @return output_folder
		'''
		return self.__output_folder

	################## PROCESS FUNCTIONS ########################

	def RestartCamera(self):
		'''
		 @brief Restart Ptgrey camera
		'''
		self.objCameraLink.RestartCamera()

	def GetRawFrames(self, get_normal_frame_only=False):
		'''
		 @brief Get new raw frames (sl and normal frame)

		 @param get_normal_frame_only (True/False - capture only a single frame (not with structured light) (default=False))
		 	If True, then sl_frame will be returned as None.

		 @return original_frame, original_sl_frame
		'''
		original_frame, original_sl_frame = self.objCameraLink.GetFrame(get_normal_frame_only=get_normal_frame_only)
		return original_frame, original_sl_frame

	def GetProcessedFrame(self, original_frame=None, original_sl_frame=None, draw_detected_points=False):
		'''
		 @brief Get new frame (both original frame and structured light (SL) frame), 
		 	and process it using computer vision techniques. Records original frames.

		 @param original_frame (May use own original_frame, if original_frame=None then newest original_frame from camera will be taken (default=None))
		 @param original_sl_frame (May use own original_sl_frame, if original_sl_frame=None then newest original_sl_frame from camera will be taken (default=None))
		 @param draw_detected_points ((True/False - (default=False))

		 @return (original_frame, original_sl_frame, frame_un, delta_frame, keypoints, descriptors)
		 	(original_frame = original frame
		 	original_sl_frame = original SL frame
		 	frame_un = undistorted (and downsized) original frame 
		 	delta_frame = frame with highlighted points above threshold
		 	keypoints = all point positions (2D numpy array = each row is a point position [x, y])
		 	descriptors = all point descriptors)
		'''
		if not(isinstance(original_frame, np.ndarray)) or not(isinstance(original_sl_frame, np.ndarray)):
			original_frame, original_sl_frame = self.GetRawFrames()

		##########################################
		#----------- PROCESS FRAME --------------#
		compute_descriptors = not(self.GetUsingBlockMatching())
		delta_frame, keypoints, descriptors, frame_un, sl_frame_un = self.GetPointList(original_frame, original_sl_frame, compute_descriptors=compute_descriptors, draw=draw_detected_points, crop_frames=self.__crop_frames, crop_frame_divisor=self.__delta_fan_angle_divisor)
		##########################################
		return (original_frame, original_sl_frame, frame_un, delta_frame, keypoints, descriptors)

	def ProcessHeading(self, frame_un, delta_frame, keypoints, draw_heading=False, draw_hough_lines=False, print_hough_positions=False):
		'''
		 @brief Compute heading.

		 @param frame_un (undistorted original frame without structured light points.)
		 @param delta_frame (frame with highlighted points above threshold)
		 @param keypoints (all point positions (2D numpy array = each row is a point position [x, y]))
		 @param draw_heading (draw heading on returned undistorted frame)
		 @param print_hough_positions (print hough line positions (rho, theta) (default=False))
		 
		 @return boundary_error, heading_error, rho, theta, edgel_map, hough_frame (Return: boundary_error, heading_error = Possible errors (None if no error and boundary_error is dominant), rho = heading distance, theta = heading direction in radian degrees, 
		 	edgel_map = (undistorted edgel_map, colored with drawings if draw_heading==True)
		 	hough_frame = (undistorted hough lines map colored with drawings if draw_hough_lines==True))
		'''
		try: # Try to find boundary hough lines
			hough_frame, edgel_map, boundary_hough_lines = self.GetBoundaryHoughLines(frame_un, delta_frame, keypoints, draw=(draw_hough_lines or draw_heading), print_hough_positions=print_hough_positions)
		except DroneVisionError, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
			return err, err, None, None, frame_un, delta_frame
		try: # Try to find heading
			heading, tip_or_root_heading, tip_or_root_detected, edgel_map = self.ComputeHeading(edgel_map, boundary_hough_lines, draw_possible_edge_headings=draw_heading, draw_headings=draw_heading)
			if tip_or_root_detected:
				heading = self.ProcessDetectionOfTipOrRoot(tip_or_root_heading)
			return None, None, heading[0], heading[1], edgel_map, hough_frame
		except DroneVisionError, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
			return None, err, None, None, edgel_map, hough_frame

	def ProcessDetectionOfTipOrRoot(self, tip_or_root_heading):
		'''
		 @brief Compute necessary action when tip or root is detected.

		 @param tip_or_root_heading

		 @return heading (rho, theta)
		'''
		if self.GetMovingTowardsTipOrRoot(): # True for moving towards the tip
			self.SetMovingTowardsRoot()
			rho 	= tip_or_root_heading[0]
			theta 	= tip_or_root_heading[1] + np.pi/2
		else:
			#### FINISHED ######
			self.SetMovingTowardsTip()
			rho 	= 0.0
			theta 	= 0.0
		heading = (rho,theta)
		return heading

	def ProcessStereopsis(self, delta_frame_l_shape, delta_frame_r_shape, keypoints_l, point_desc_l, keypoints_r, point_desc_r, draw_matches=False):
		'''
		 @brief Compute 3D coordinate points in world frame.

 		 @param delta_frame_l_shape
 		 @param delta_frame_r_shape
		 @param keypoints_l
		 @param point_desc_l
		 @param keypoints_r
		 @param point_desc_r
		 @param draw_matches (default=False)

		 if draw_matches:
		 	@return error, points3D, matches_frame (Returns: error = possible error (None if no error), points3D = computed 3D points, matches_frame = frame with drawn matches)
		 else:
		 	@return error, points3D, None (Returns: error = possible error (None if no error), points3D = computed 3D points, matches_frame=None)
		'''
		try:
			points3D = self.Get3DPoints(delta_frame_l_shape, delta_frame_r_shape, keypoints_l, point_desc_l, keypoints_r, point_desc_r, draw=draw_matches)
			if draw_matches:
				return None, points3D[0], points3D[1] # points3D = (points3D, matches_frame)
			return None, points3D, None # Else
		except DroneVisionError, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
			return err, None, None

	def CheckDroneVisionFinished(self):
		'''
		 @Check if drone vision is finished.
		 	Stops if:
		 		n_frames in settings exceed number of frames processed (as long as n_frames > 0).
		 		or - processed frames exceeds n_frames in pre-recorded video.

		 @return True/False - True = Finished, False = Not Finished
		'''
		stop = False
		frame_i  = self.objCameraLink.GetFrameNumber()
		n_frames = self.objCameraLink.GetTotalFrames()
		if (frame_i >= self.GetSettings('CAMERA', 'n_frames') and self.GetSettings('CAMERA', 'n_frames') > 0) or frame_i >= n_frames:
			stop = True
		return stop #else
