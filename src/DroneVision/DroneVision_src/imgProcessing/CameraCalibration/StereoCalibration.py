'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''
import cv2
import numpy as np
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape
from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, RealTimePlot
from src.DroneVision.DroneVision_src.hardware.PyQtImage import PyQtImage
from CameraCalibration import CameraCalibration
from src.bin.SaveParameters import SaveParameters

'''
 @brief Class for calibrating the stereo vision system.

 @param me_master (True if this instance is master, False for slave)
 @param settings_inst (CALIB settings)
 @param reset (True/False)
 @param plot_figure (optional plot figure (default=None))
 @param use_PyQt (default=True)
'''
class StereoCalibration():
	def __init__(self, me_master, settings_inst, reset, plot_figure=None, use_PyQt=True):
		'''CONSTRUCTOR'''
		self.__saveParameters 			= SaveParameters(settings_inst.GetSettings('calib_save_folder'), settings_inst.GetSettings('calib_save_fname_stereo'), settings_inst.GetSettings('save_calib_param_to_json'))
		self.__leftCameraCalibration 	= CameraCalibration(settings_inst, settings_inst.GetSettings('calib_img_folder_left_cam'), settings_inst.GetSettings('calib_save_fname_left_cam'), reset, plot_figure=plot_figure)
		self.__rightCameraCalibration 	= CameraCalibration(settings_inst, settings_inst.GetSettings('calib_img_folder_right_cam'), settings_inst.GetSettings('calib_save_fname_right_cam'), reset, plot_figure=plot_figure)
		self.__show_chessboard_img 		= settings_inst.GetSettings('calib_show_imgs')
		self.__baseline 				= settings_inst.GetSettings('baseline')
		self.__stereo_calib_reset 		= reset
		self.__me_master 				= me_master
		self.__stereo_calibrated 		= False
		self.__plot_figure 				= plot_figure
		self.__use_PyQt 				= use_PyQt
		self.__calib_params 			= {}

	def CalibrateStereoVisionSystem(self, force_calibration=False, default_frame_shape=(-1,-1)):
		'''
		 @brief Calibrate stereo vision system, with full calibration of cameras as well.

		 @param force_calibration (True/False for forcing new full calibration)
		 @param default_frame_shape (Default desired frame shape of processed frames. 
		 	Given as a tuple of (height, width). 
		 	The rectification vectors will automatically be adjusted to incoming frame shapes (only ones for a new shape), but it is time consuming to compute. 
		 	Set (-1,-1) to not change the precomputed intrinsic parameters (default))
		'''
		self.__leftCameraCalibration.CalibrateCameraDistortion(force_calibration=force_calibration)
		self.__rightCameraCalibration.CalibrateCameraDistortion(force_calibration=force_calibration)
		self.__stereo_calibrated = True
		new_calibration = False
		if not(self.LoadStereoParameters()) or self.__stereo_calib_reset or force_calibration:
			new_calibration = True
			self.StereoCalibrate()
			self.StereoRectify()
			self.SaveStereoParameters()
		self.InitUndistortRectifyMapStereo()
		if new_calibration:
			self.ShowTestCalibImage()
		if default_frame_shape[0] > 0:
			self.SetIntrinsicStereoScale(default_frame_shape)

	def GetNewRealTimePlot(self):
		'''
		 @brief Get new realtime plot figure

		 @return realtime plot figure
		'''
		if self.__plot_figure != None:
			realTimePlot = self.__plot_figure
			realTimePlot(reset=True)
		else:
			if self.__use_PyQt:
				realTimePlot = PyQtImage(True)
			else:
				realTimePlot = RealTimePlot()
		return realTimePlot

	def ShowTestCalibImage(self):
		'''
		 @brief Show test image in plot
		'''
		if self.__show_chessboard_img:
			touple_frames = []
			if self.__me_master:
				side_txt  		= 'left'
				test_img_fname 	= self.__leftCameraCalibration.GetDistorionCalibImages()[0]
			else:
				side_txt  		= 'right'
				test_img_fname = self.__rightCameraCalibration.GetDistorionCalibImages()[0]

			test_img 				= GetImage(test_img_fname)
			
			headline 				= '[{0}] before shape {1}'.format(side_txt, test_img.shape)
			touple_frames.append((headline, test_img))
			
			test_und_img 			= self.Undistort(test_img)
			headline 				= '[{0}] After undistort shape {1}'.format(side_txt, test_und_img.shape)
			touple_frames.append((headline, test_und_img))
			
			realTimePlot = self.GetNewRealTimePlot()
			realTimePlot(touple_frames, 'calibration_result')

	def AssertStereoCalibrated(self):
		'''
		 @brief Assert that the stereo vision system is calibrated.
		 	Raises Exception if it is not calibrated.
		'''
		if not(self.GetStereoCalibrated()):
			raise Exception('Stereo is not calibrated. Run CalibrateStereoVisionSystem().')

	def CheckIntrinsicStereoScale(self, frame_size):
		'''
		 @brief Check intrinsic stereo scale

		 @return True/False
		'''
		return self.__leftCameraCalibration.CheckIntrinsicScale(frame_size)

	def GetLeftCameraCalibrationInstance(self):
		'''
		 @brief Get left camera calibration instance

		 @return leftCameraCalibration
		'''
		return self.__leftCameraCalibration

	def GetRightCameraCalibrationInstance(self):
		'''
		 @brief Get right camera calibration instance

		 @return rightCameraCalibration
		'''
		return self.__rightCameraCalibration

	def GetBaseline(self):
		'''
		 @brief Get baseline between cameras in mm

		 @return baseline
		'''
		return float(self.__baseline)

	def GetPixelBaseline(self):
		'''
		 @brief Get baseline between camers in pixel units
		'''
		self.AssertStereoCalibrated()
		return self.__calib_params['P2'][0,3]*-1 # Projection matrix give negated baseline seen from the right camera.

	def GetFocalLength(self):
		'''
		 @brief Get original focal length in mm

		 @return focal_length
		'''
		return self.__leftCameraCalibration.GetFocalLength()

	def GetPixelFocalLength(self):
		'''
		 @brief Get focal length in camera pixel units

		 @return f_x, f_y, f_z
		'''
		self.AssertStereoCalibrated()
		f_x = self.__calib_params['P1'][0,0]
		f_y = self.__calib_params['P1'][1,1]
		f_z = (f_x + f_y)/2.0
		return f_x, f_y, f_z

	def GetProjectionMatrices(self):
		'''
		 @brief Get projection matrices (P1 and P2)

		 @return P1, P2
		'''
		return self.__calib_params['P1'], self.__calib_params['P2']

	def GetDisparityToDepthMatrix(self):
		'''
		 @brief Get disparity to depth transformation matrix (Q)

		 @return Q
		'''
		return self.__calib_params['Q']

	def GetStereoCalibrated(self):
		'''
		 @brief Check if stereo vision system is calibrated

		 @return True/False
		'''
		return self.__stereo_calibrated

	def AssertSameStereoSize(self):
		'''
		 @brief Assert that both cameras have same image dimensions
		'''
		left_imageSize 	 = self.__leftCameraCalibration.GetImageSize()
		right_imageSize  = self.__rightCameraCalibration.GetImageSize()
		if not(left_imageSize[0] == right_imageSize[0]) or not(left_imageSize[1] == right_imageSize[1]):
			raise ValueError('Left and right image dimensions do not match!' )

	def StereoCalibrate(self):
		'''
		 @brief Calibrates the stereo camera first, and then computes rectification transforms for each head of a calibrated stereo camera.
		 	Computes rotation matrix (R), translation vector (T), essential matrix (E) and fundamental matrix (F)
		'''
		self.AssertSameStereoSize()
		cameraMatrix1, distCoeffs1 = self.__leftCameraCalibration.GetIntrinsicParameters()
		cameraMatrix2, distCoeffs2 = self.__rightCameraCalibration.GetIntrinsicParameters()
		objectPoints = self.__leftCameraCalibration.GetObjectPoints()
		imagePoints1 = self.__leftCameraCalibration.GetImagePoints()
		imagePoints2 = self.__rightCameraCalibration.GetImagePoints()
		imageSize 	 = self.__leftCameraCalibration.GetImageSize()
		stereocalib_criteria 	= (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
		#stereocalib_flags 		= cv2.CALIB_FIX_ASPECT_RATIO | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_SAME_FOCAL_LENGTH | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5
		#stereocalib_flags 		= cv2.CALIB_FIX_INTRINSIC | cv2.CALIB_SAME_FOCAL_LENGTH
		stereocalib_flags  		= cv2.CALIB_FIX_INTRINSIC | cv2.CALIB_ZERO_DISPARITY | cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_FIX_K3 | cv2.CALIB_FIX_K4 | cv2.CALIB_FIX_K5
		retval, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(objectPoints, \
																										imagePoints1, \
																										imagePoints2, \
																										cameraMatrix1, \
																										distCoeffs1, \
																										cameraMatrix2, \
																										distCoeffs2, \
																										(imageSize[1], imageSize[0]), \
																										criteria=stereocalib_criteria, \
																										flags=stereocalib_flags)
		if not(retval):
			raise Exception('Stereo calibration failed!')
		# Store params in dictionary
		self.__calib_params['cameraMatrix1'] 		= cameraMatrix1
		self.__calib_params['distCoeffs1'] 			= distCoeffs1
		self.__calib_params['cameraMatrix2'] 		= cameraMatrix2
		self.__calib_params['distCoeffs2'] 			= distCoeffs2
		self.__calib_params['R'] 					= R
		self.__calib_params['T'] 					= T
		self.__calib_params['E'] 					= E
		self.__calib_params['F'] 					= F

	def StereoRectify(self, frame_size=None, rectify_scale=0.0):
		'''
		 @brief Rectify the stereopsis system.
			Computes rectification transform (rotation matrices - 3x3) (R), projection matrices 3x4 (P) and disparity to depth mapping matrix 4x4 (Q)
		 	rectify_scale:  0 = full crop, 1 = no crop
			If rectify_scale = 1, all pixels are retained with some extra black images.
			If rectify_scale = 0, it returns undistorted image with minimum unwanted pixels.

		 @param frame_size ((height, width) If None, then stored left frame size is used (default=None))
		 @param rectify_scale (default=0.0)
		'''
		if not(isinstance(frame_size, tuple)) and not(isinstance(frame_size, list)):
			frame_size = self.__leftCameraCalibration.GetImageSize()
		if not(self.CheckIntrinsicStereoScale(frame_size)):
			self.__leftCameraCalibration.RectifyCamera(frame_size)
			self.__rightCameraCalibration.RectifyCamera(frame_size)
			self.StereoCalibrate()
		self.__calib_params['R'], self.__calib_params['T'] = self.ComputeTranslationAndRotationMatrices()
		self.__calib_params['R1'], self.__calib_params['R2'], self.__calib_params['P1'], self.__calib_params['P2'], self.__calib_params['Q'], self.__calib_params['roi1'], self.__calib_params['roi2'] = cv2.stereoRectify(self.__calib_params['cameraMatrix1'], self.__calib_params['distCoeffs1'], self.__calib_params['cameraMatrix2'], self.__calib_params['distCoeffs2'], (frame_size[1], frame_size[0]), self.__calib_params['R'], self.__calib_params['T'], alpha=rectify_scale)

	def InitUndistortRectifyMapStereo(self):
		'''
		 @brief Compute rectification maps
		'''
		frame_size = self.__leftCameraCalibration.GetImageSize()
		self.__left_rectify_maps 	= cv2.initUndistortRectifyMap(self.__calib_params['cameraMatrix1'], self.__calib_params['distCoeffs1'], self.__calib_params['R1'], self.__calib_params['P1'], (frame_size[1], frame_size[0]), cv2.CV_16SC2)
		self.__right_rectify_maps	= cv2.initUndistortRectifyMap(self.__calib_params['cameraMatrix2'], self.__calib_params['distCoeffs2'], self.__calib_params['R2'], self.__calib_params['P2'], (frame_size[1], frame_size[0]), cv2.CV_16SC2)

	def ComputeTranslationAndRotationMatrices(self):
		'''
		 @brief Compute the translation vector (T) and rotation vector (R) on perfectly horizontally aligned cameras.

		 @return R, T
		'''
		R 		= np.eye(3) # Perfectly aligned cameras
		T 		= np.zeros((3,1))
		T_x 	= self.GetBaseline() # horizontal baseline in mm
		T[0,0] 	= -T_x
		return R, T

	def SetIntrinsicStereoScale(self, frame_size):
		'''
		 @brief Set new intrinsic scale parameters for a new frame shape (if it is different from the current parameters). 
		 	Calibration is invariant to scale, but intrinsic parameters are not.

		 @param frame_size (Tuple as (height, width))
		'''
		if not(self.CheckIntrinsicStereoScale(frame_size)):
			self.StereoRectify(frame_size)
			self.InitUndistortRectifyMapStereo()

	def Undistort(self, frame):
		'''
		 @brief Stereo undistorting

		 @return undistorted frame
		'''
		self.AssertStereoCalibrated()
		if not(self.CheckIntrinsicStereoScale(GetShape(frame))):
			self.SetIntrinsicStereoScale(GetShape(frame))
		if self.__me_master:
			und_frame = cv2.remap(frame, self.__left_rectify_maps[0], self.__left_rectify_maps[1], cv2.INTER_LANCZOS4)
		else:
			und_frame = cv2.remap(frame, self.__right_rectify_maps[0], self.__right_rectify_maps[1], cv2.INTER_LANCZOS4)
		return self.CropUndistortedFrame(und_frame)

	def CropUndistortedFrame(self, und_frame):
		'''
		 @brief Crop undistorted frame

		 @param und_frame

		 @return und_frame Cropped undistorted frame
		'''
		if self.__me_master:
			x, y, w, h 	= self.__calib_params['roi1']
		else:
			x, y, w, h 	= self.__calib_params['roi2']
		und_frame 	= und_frame[y:y+h, x:x+w]
		return und_frame

	def SaveStereoParameters(self):
		'''
		 @brief Save stereo parameters for later use.
		'''
		self.__saveParameters.Save(self.__calib_params)

	def LoadStereoParameters(self):
		'''
		 @brief Load stereo parameters

		 @return True/False - stereo parameters loaded successfully.
		'''
		ok, self.__calib_params = self.__saveParameters.Load()
		return ok