'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

import time
from LaserLink import LaserLink
from PtGrey.PtGrey import PtGrey

'''
 @brief Class for handling the Camera link.

 @param me_master (True/False - True = master, False = slave)
 @param camera_settings_inst (settings['CAMERA'])
 @param laser_settings_inst (settings['LASER'])
'''
class CameraLink(LaserLink, PtGrey):
	def __init__(self, me_master, camera_settings_inst, laser_settings_inst):
		'''CONSTRUCTOR'''
		self.__frame_i 				= 0
		self.__use_color_frames 	= camera_settings_inst.GetSettings('ptg_use_color_frames')
		self.__auto_PtGrey_config 	= camera_settings_inst.GetSettings('auto_camera_config')
		manual_triggering 			= camera_settings_inst.GetSettings('manual_triggering')
		ptg_recv_frame_timeout 		= camera_settings_inst.GetSettings('ptg_recv_frame_timeout')
		if manual_triggering and not(me_master): # Force slave to wait forever for a frame when user trigs it manually.
			ptg_recv_frame_timeout 	= -1
			manual_triggering 		= False

		LaserLink.__init__(self, laser_settings_inst.GetSettings('laser_triggerPin'))
		PtGrey.__init__(self, camera_settings_inst.GetSettings('ptgrey_library'), camera_settings_inst.GetSettings('camera_triggerPin'), \
			camera_settings_inst.GetSettings('ptg_triggerPin'), \
			manual_triggering, \
			ptg_recv_frame_timeout)

		self.StartCapture()
		self.ConfigureCameraSettings(set_auto=self.__auto_PtGrey_config, use_color_frames=self.__use_color_frames) # Set initial values

	def ConfigureCameraSettings(self, set_auto=False, use_color_frames=False):
		'''
		 @brief configure camera settings with fixed parameters

		 @param set_auto (Set camera settings to be configured automatically by the camera (True/False - default=False))
		 @param use_color_frames (True/False for using color frames (default=False))
		'''
		if not(self.CheckCapturing()):
			if use_color_frames:
				self.SetFormat7Configuration(mode=self.GetFc2().MODE_0, pixel_format=self.GetFc2().PIXEL_FORMAT_RGB8)
			else:
				self.SetFormat7Configuration(mode=self.GetFc2().MODE_0, pixel_format=self.GetFc2().PIXEL_FORMAT_RAW8)
			self.SetConfiguration(num_buffers=3)
		self.SetFrameRate(32.0)

		if use_color_frames:
			''' FOR RGB8 SETTINGS '''
			self.SetGain(7.3, auto=set_auto)
			self.SetShutter(50.52, auto=set_auto)
			self.SetBrightness(2.0)
			self.SetAutoExposure(1.34, auto=set_auto)
			self.SetGamma(1.5, auto=set_auto)
			self.SetWhiteBalance(1536, 0, auto=set_auto)
		else:
			''' FOR RAW8 SETTINGS '''
			self.SetGain(28.09, auto=set_auto)
			self.SetShutter(109.61, auto=set_auto)
			self.SetBrightness(3.0)
			self.SetAutoExposure(0.923, auto=set_auto)
			self.SetGamma(1.5, auto=set_auto)
			self.SetWhiteBalance(1536, 0, auto=set_auto)

	def GetTotalFrames(self):
		'''
		 @brief Get total frames captured. (to correspond with VideoLink class)

		 @return frame_i + 1
		'''
		return self.__frame_i + 1

	def GetFrameProperties(self):
		'''
		 @brief Get frame properties such as fps, width, length

		 @return fps, width, height
		'''
		format7config = self.SetFormat7Configuration()
		width 	= format7config['width']
		height 	= format7config['height']
		frame_rate_property = self.SetFrameRate()
		frame_rate = frame_rate_property['abs_value']
		return frame_rate, width, height

	def GetFrame(self, get_normal_frame_only=False):
		'''
		 @brief Get new frame and structured light (sl) from camera
		 	Sets frame properties at the same time.

		 @param get_normal_frame_only (True/False - capture only a single frame (not with structured light) (default=False))
		 	If True, then sl_frame will be returned as None.

		 @return frame, sl_frame
		'''
		frame = self.CaptureFrame()

		if not(get_normal_frame_only):
			self.LaserON()
			if self.CheckManualTriggering():
				print 'Turn laser on if necessary..'
			sl_frame = self.CaptureFrame()
			self.LaserOFF()
		else:
			sl_frame = None

		self.__frame_i += 1

		return frame, sl_frame

	def GetFrameNumber(self):
		'''
		 @brief Get current frame number

		 @return frame_i
		'''
		return self.__frame_i

	def StopCamera(self):
		'''
		 @brief Stop camera
		'''
		self.StopCapture()

	def RestartCamera(self):
		'''
		 @brief Restart PtGrey camera
		'''
		self.RestartCapture()
		self.ConfigureCameraSettings(set_auto=self.__auto_PtGrey_config, use_color_frames=self.__use_color_frames) # Set initial values

	def __del__(self):
		'''DESTRUCTOR'''
		self.StopCamera()
		PtGrey.__del__()