'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''
import cv2
import numpy as np
import glob, os
from src.bin.SaveParameters import SaveParameters
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import CheckGrayScale, GetShape
from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, RealTimePlot
from src.DroneVision.DroneVision_src.hardware.PyQtImage import PyQtImage

'''
 @brief Camera calibration class. Follow steps on the opencv page:
 	http://docs.opencv.org/3.1.0/dc/dbb/tutorial_py_calibration.html
 	The cailbration folder should contain at least 5-20 images of a chess board. 
 	The chess board must be placed at the operational distance with all extrinsic camera parameters fixed.
 	Please use the chessboard provided with this scipt. Number of rows and columns for this chessboard is set to 6x9.
 	Any new chessboard requires new settings. Set calib_chess_rows and calib_chess_columns according to the used calibration chessboard.

 	Step 1: Set camera calibration by running: self.RunCameraCalibration()
 	Step 2: Undistort each received frame by: frame = self.Undistort(frame)

 @param settings_inst - Calibration settings instance with following parameters:
 	 - calib_img_type Image types (jpg, tif...)
 	 - calib_save_folder Save folder for calibration parameters.
 	 - calib_show_imgs Show chessboard images during process.
 	 - calib_reset Reset calibration in any case.
 	 - calib_chess_rows Number of rows of the chessboard (innen chessboard)
 	 - calib_chess_columns Number of columns of the chessboard (innen chessboard)
 	 - save_calib_param_to_json Save calibration parameters to json readable file (False will save parameters to python pickle file)
 @param calib_folder Folder with calibration images. 
 @param calib_save_fname Filename of saved calibration parameters
 @param reset (True/False)
 @param plot_figure (optional plot figure (default=None))
 @param use_PyQt (default=True)
'''
class CameraCalibration():
	def __init__(self, settings_inst, calib_folder, calib_save_fname, reset, plot_figure=None, use_PyQt=True):
		'''CONSTRUCTOR'''
		self.__saveParameters 			= SaveParameters(settings_inst.GetSettings('calib_save_folder'), calib_save_fname, settings_inst.GetSettings('save_calib_param_to_json'))
		self.__calib_folder 			= calib_folder
		self.__focal_length 			= settings_inst.GetSettings('focal_length')
		self.__sensor_size 				= settings_inst.GetSettings('sensor_size')
		self.__image_type 				= settings_inst.GetSettings('calib_img_type')
		self.__show_chessboard_img 		= settings_inst.GetSettings('calib_show_imgs')
		self.__print_process 			= settings_inst.GetSettings('calib_print_process')
		self.__calib_chess_rows 		= settings_inst.GetSettings('calib_chess_rows')
		self.__calib_chess_columns 		= settings_inst.GetSettings('calib_chess_columns')
		self.__distortion_calibrated 	= False
		self.__calib_reset 				= reset
		self.__plot_figure 				= plot_figure
		self.__use_PyQt 				= use_PyQt
		self.__calib_params 			= {}

	def CalibrateCameraDistortion(self, force_calibration=False):
		'''
		 @brief Run camera calibration.

		 @param force_calibration (True/False for forcing new full calibration)
		'''
		self.__distortion_calibrated = True
		if not(self.LoadDistortionParameters()) or self.__calib_reset or force_calibration:
			self.InitDistortionParameters()
			self.SetDistortionCalibImages()
			self.FindChessBoardCorners()
			self.CalibrateCamera()
			self.RectifyCamera()
			self.SaveDistortionParameters()
		self.InitUndistortRectifyMap()

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
			test_img_fname 			= self.GetDistorionCalibImages()[0]
			test_img 				= GetImage(test_img_fname)
			
			headline 				= 'Before shape {0}'.format(test_img.shape)
			touple_frames.append((headline, test_img))
			
			test_und_img 			= self.Undistort(test_img)
			headline 				= 'After undistort shape {0}'.format(test_und_img.shape)
			touple_frames.append((headline, test_und_img))
			
			realTimePlot = self.GetNewRealTimePlot()
			realTimePlot(touple_frames, 'calibration_result')

	def AssertCameraCalibrated(self):
		'''
		 @brief Assert that the camera is calibrated.
		 	Raises Exception if its not calibrated.
		'''
		if not(self.GetDistortionCalibrated()):
			raise Exception('Camera is not calibrated. Run CalibrateCameraDistortion().')

	def GetFocalLength(self):
		'''
		 @brief Get original focal length in mm

		 @return focal_length
		'''
		return float(self.__focal_length)

	def GetPixelFocalLength(self, optimal_mtx=False):
		'''
		 @brief Get focal length in pixel units

		 @param optimal_mtx (True for returning optimized values of focal length (default=False))

		 @return (f_x, f_y)
		'''
		self.AssertCameraCalibrated()
		if optimal_mtx:
			f_x = self.__calib_params['optimized_intrinsic_mtx'][0,0]
			f_y = self.__calib_params['optimized_intrinsic_mtx'][1,1]
		else:
			f_x = self.__calib_params['intrinsic_mtx'][0,0]
			f_y = self.__calib_params['intrinsic_mtx'][1,1]
		return (f_x, f_y)

	def GetSensorSize(self):
		'''
		 @brief Get sensor size

		 @return sensor_size ((height, width) in mm of the sensor size.)
		'''
		return self.__sensor_size 

	def SetResetCalibration(self):
		'''
		 @brief Set reset to True for new calibration
		'''
		self.__calib_reset = True

	def GetDistortionCalibrated(self):
		'''
		 @brief Get True/False to check wether the calibration has been done.

		 @return True/False
		'''
		return self.__distortion_calibrated

	def ComputeCameraMatrix(self, frame_size, focal_length, sensor_size):
		'''
		 @brief Compute camera matrix (intrinsic matrix) from focal length and principal points in frame center.
		 		f_x, f_y, c_x, c_y are expressed in pixel units.

		 @param frame_size (height, width)
		 @param focal_length (in mm)
		 @param sensor_size (in mm as (height, width))

		 @return intrinsic_mtx (numpy array as [[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]])
		'''
		f_x = (focal_length*float(frame_size[1]))/sensor_size[1]
		f_y = (focal_length*float(frame_size[0]))/sensor_size[0]
		c_x = frame_size[1]/2.0
		c_y = frame_size[0]/2.0
		intrinsic_mtx = np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]])
		return intrinsic_mtx

	def InitDistortionParameters(self):
		'''
		 @brief Set initial calibration parameters.
		'''
		# termination criteria
		self.__criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
		
		# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
		self.__objp 		= np.zeros((self.__calib_chess_columns*self.__calib_chess_rows,3), np.float32)
		self.__objp[:,:2] 	= np.mgrid[0:self.__calib_chess_rows,0:self.__calib_chess_columns].T.reshape(-1,2)

		# Arrays to store object points and image points from all the images.
		self.__calib_params['objpoints'] = [] # 3d point in real world space
		self.__calib_params['imgpoints'] = [] # 2d points in image plane.

	def SetDistortionCalibImages(self):
		'''
		 @brief Set all distortion calibration image filenames
		'''
		if not(os.path.isdir('./'+self.__calib_folder)):
			raise Exception('Calibration folder does not exist: ' + self.__calib_folder)
		self.__calib_img_fnames = glob.glob(self.__calib_folder+'*.'+self.__image_type)
		if len(self.__calib_img_fnames) == 0:
			raise Exception('No calibration images present in calibration folder: {0}, of type: {1}'.format(self.__calib_folder, self.__image_type))

	def GetDistorionCalibImages(self):
		'''
		 @brief Get all distortion calibration image filenames

		 @return calib_img_fnames
		'''
		self.SetDistortionCalibImages()
		return self.__calib_img_fnames

	def FindChessBoardCorners(self):
		'''
		 @brief Find chessboard matches and append to imgpoints.
		'''
		if self.__show_chessboard_img:
			realTimePlot = self.GetNewRealTimePlot()

		ret_calib 	= False
		for i in range(len(self.__calib_img_fnames)):
			fname 		= self.__calib_img_fnames[i]
			img 		= GetImage(fname, gray=False)
			gray 		= CheckGrayScale(img)
			gray_shape 	= GetShape(gray)
			if i > 0:
				if not(self.CheckIntrinsicScale(gray_shape)):
					raise ValueError('Calibration image dimensions do not match!')
			else:
				self.SetScale(gray_shape)

			# Find the chess board corners
			flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
			ret, corners = cv2.findChessboardCorners(gray, (self.__calib_chess_rows,self.__calib_chess_columns), flags=flags)

			# If found, add object points, image points (after refining them)
			print_msg = '{0}/{1} images processed'.format(i+1, len(self.__calib_img_fnames))
			if ret:
				self.PrintCalibrationProcess(print_msg+' - {0} was successfull'.format(fname))
				ret_calib = True
				self.__calib_params['objpoints'].append(self.__objp)

				corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), self.__criteria)
				self.__calib_params['imgpoints'].append(corners)

				if self.__show_chessboard_img:
					cv2.drawChessboardCorners(img, (self.__calib_chess_rows,self.__calib_chess_columns), corners, ret)
					realTimePlot(reset=True)
					realTimePlot([(fname[len(self.__calib_folder):], img)], 'calibration_frames')
			else:
				self.PrintCalibrationProcess(print_msg+' - {0} was not successfull'.format(fname))
				if self.__show_chessboard_img:
					realTimePlot(reset=True)
					realTimePlot([('Failed: '+fname[len(self.__calib_folder):], gray)], 'calibration_frames')

		if not(ret_calib):
			err_msg = 'None of the chessboard calibration frames could be used for calibration. Folder = {0}'.format(self.__calib_folder)
			raise Exception(err_msg)

	def PrintCalibrationProcess(self, msg):
		'''
		 @brief Print process message if told to.
		
		 @param msg
		'''
		if self.__print_process or self.__show_chessboard_img:
			print msg

	def CalibrateCamera(self, frame_size=None):
		'''
		 @brief Compute the camera matrix, distortion coefficients, rotation and translation vectors.

		 @param frame_size (height. width)
		'''
		if not(isinstance(frame_size, tuple)) and not(isinstance(frame_size, list)):
			frame_size = GetShape(GetImage(self.__calib_img_fnames[0]))
		if self.__focal_length != None and self.__sensor_size != None: 
			#flags 	= cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_ASPECT_RATIO
			flags 	= cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_PRINCIPAL_POINT
			self.__calib_params['intrinsic_mtx'] = self.ComputeCameraMatrix(frame_size, self.__focal_length, self.__sensor_size)
			retval, self.__calib_params['intrinsic_mtx'], self.__calib_params['distortion_coeffs'], rvecs, tvecs = cv2.calibrateCamera(self.__calib_params['objpoints'], self.__calib_params['imgpoints'], (frame_size[1], frame_size[0]), self.__calib_params['intrinsic_mtx'], None, flags=flags)
		else:
			retval, self.__calib_params['intrinsic_mtx'], self.__calib_params['distortion_coeffs'], rvecs, tvecs = cv2.calibrateCamera(self.__calib_params['objpoints'], self.__calib_params['imgpoints'], (frame_size[1], frame_size[0]), None, None)
		if not(retval):
			raise Exception('Camera calibration failed!')

	def RectifyCamera(self, frame_size=None, rectify_scale=0.0, force_recalibration_of_intrinsic_mtx=False):
		'''
		 @brief The calibration is done using the original frame size.
		 	Set the undistortion parameters according to the used frame size which may be downsampled.
		 	Set ret (True/False), mtx, dist, rvecs, tvecs, optimized_intrinsic_mtx
			rectify_scale:  0 = full crop, 1 = no crop
			If rectify_scale = 1, all pixels are retained with some extra black images.
			If rectify_scale = 0, it returns undistorted image with minimum unwanted pixels.
		

		 @param frame_size ((height, width) If None, then stored frame size is used (default=None))
		 @param rectify_scale (default=0.0)
		 @param force_recalibration_of_intrinsic_mtx (Force recalibration of intrinsic matrix. Could be done instead of scaling it used a new frame size, which should be not as accurate. (default=False))
		'''
		if not(isinstance(frame_size, tuple)) and not(isinstance(frame_size, list)):
			frame_size = self.GetImageSize()
		if not(self.CheckIntrinsicScale(frame_size)):
			if force_recalibration_of_intrinsic_mtx:
				self.CalibrateCamera(frame_size)
			else:
				divisor = self.GetScaleDivisor(frame_size)
				self.__calib_params['intrinsic_mtx'] = self.ScaleCameraMatrix(self.__calib_params['intrinsic_mtx'], divisor)
			self.SetScale(frame_size)
		self.__calib_params['optimized_intrinsic_mtx'], self.__calib_params['roi'] = cv2.getOptimalNewCameraMatrix(self.__calib_params['intrinsic_mtx'],self.__calib_params['distortion_coeffs'], (frame_size[1], frame_size[0]), rectify_scale)

	def InitUndistortRectifyMap(self, R=None):
		'''
		 @brief Compute rectification map

		 @param R (optional camera rotation computed from stereoRectify)
		'''
		frame_size = self.GetImageSize()
		self.__mapx, self.__mapy = cv2.initUndistortRectifyMap(self.__calib_params['intrinsic_mtx'],self.__calib_params['distortion_coeffs'], R, self.__calib_params['optimized_intrinsic_mtx'], (frame_size[1], frame_size[0]), cv2.CV_16SC2)

	def CheckIntrinsicScale(self, frame_size):
		'''
		 @brief Check if the intrinsic scaling is set according to the used frame scale.

		 @param frame_size ((height, width) of the preferred frame scale)

		 @return True/False
		'''
		self.AssertCameraCalibrated()
		ok = False
		if self.__calib_params['height'] == frame_size[0] and self.__calib_params['width'] == frame_size[1]:
			ok = True
		return ok

	def GetScaleDivisor(self, frame_size):
		'''
		 @brief Get scale divisor to use in ScaleCameraMatrix

		 @param frame_size ((height, width) frame dimensions)

		 @return divisor
		'''
		divisor = (self.__calib_params['height']/float(frame_size[0]) + self.__calib_params['width']/float(frame_size[1]))/2.0
		return divisor

	def SetScale(self, frame_size):
		'''
		 @brief Set the new scale size

		 @param frame_size ((height, width) frame dimensions) 
		'''
		self.__calib_params['height'] 	= frame_size[0]
		self.__calib_params['width'] 	= frame_size[1]

	def ScaleCameraMatrix(self, intrinsic_mtx, divisor):
		'''
		 @brief Scale camera matrix (intrinsic values) to appropriate image size.
		 	The calibration should be done on a raw frame (not down/up sampled), which means the intrinsic values must be scaled thereafter.
		 	The distortion coeffisients ('dist') do not depend on the scene view, so they are independent of scaling.

		 @param intrinsic_mtx (intrinsic matrix)
		 @param divisor (scaling divisor)

		 @return scaled_intrinsic_mtx
		'''
		scaled_intrinsic_mtx 		= intrinsic_mtx/divisor
		scaled_intrinsic_mtx[2,2] 	= 1.0
		return scaled_intrinsic_mtx

	def GetIntrinsicParameters(self, with_optimized_camera_mtx=True):
		'''
		 @brief Get intrinsic parameters for the camera.

		 @param with_optimized_camera_mtx (True/False for getting optimized camera matrix (default=False))

		 @return cameraMatrix, distCoeffs
		'''
		self.AssertCameraCalibrated()
		if with_optimized_camera_mtx:
			return self.__calib_params['optimized_intrinsic_mtx'].copy(),self.__calib_params['distortion_coeffs'].copy()
		else:
			return self.__calib_params['intrinsic_mtx'].copy(),self.__calib_params['distortion_coeffs'].copy()

	def GetObjectPoints(self):
		'''
		 @brief Get object points

		 @return ObjectPoints
		'''
		self.AssertCameraCalibrated()
		return list(self.__calib_params['objpoints'])

	def GetImagePoints(self):
		'''
		 @brief Get image points

		 @return ImagePoints
		'''
		self.AssertCameraCalibrated()
		return list(self.__calib_params['imgpoints'])

	def GetImageSize(self):
		'''
		 @brief Get image size

		 @return ImageSize (height, width)
		'''
		self.AssertCameraCalibrated()
		return (int(self.__calib_params['height']), int(self.__calib_params['width']))

	def Undistort(self, frame):
		'''
		 @brief Undistort frame using the remapping method.
		 	Should give same result as Undistort()

		 @param frame

		 @return Undistorted frame
		'''
		self.AssertCameraCalibrated()
		if not(self.CheckIntrinsicScale(GetShape(frame))):
			self.RectifyCamera(GetShape(frame))
			self.InitUndistortRectifyMap()
		und_frame = cv2.remap(frame, self.__mapx, self.__mapy, cv2.INTER_LINEAR)
		return self.CropUndistortedFrame(und_frame)

	def CropUndistortedFrame(self, und_frame):
		'''
		 @brief Crop undistorted frame

		 @param und_frame

		 @return und_frame Cropped undistorted frame
		'''
		x, y, w, h 	= self.__calib_params['roi']
		und_frame 	= und_frame[y:y+h, x:x+w]
		return und_frame

	def SaveDistortionParameters(self):
		'''
		 @brief Save calibration parameters for later use.
		'''
		self.__saveParameters.Save(self.__calib_params)

	def LoadDistortionParameters(self):
		'''
		 @brief Load calibration parameters

		 @return True/False - calibration parameters loaded successfully.
		'''
		ok, self.__calib_params = self.__saveParameters.Load()
		return ok
