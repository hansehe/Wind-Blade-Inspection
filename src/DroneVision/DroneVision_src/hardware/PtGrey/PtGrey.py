'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

from PtGreyFLIR import PtGreyFLIR
from PtGreyJordens import PtGreyJordens

'''
 @brief Selection between either the PtGrey api from FLIR (official) or the Jordens (pyflycapture2) github api.

 @param ptgrey_lib_tag (Tag for selecting which library to use. Options - FLIR library: 'FLIR', Jordens library: 'Jordens'. Jordens library is set as default.)
 @param *args, **kwargs (All necessary parameters for the inherited libraries. Jordens and FLIR ptgrey library are identical regarding parameters and usage.)
'''
class PtGrey(object):
	_PtGrey_options = {PtGreyFLIR._PtGrey_lib_tag: PtGreyFLIR, PtGreyJordens._PtGrey_lib_tag: PtGreyJordens} # Append possible ptgrey libraries 

	def __init__(self, ptgrey_lib_tag=PtGreyJordens._PtGrey_lib_tag, *args, **kwargs):
		if ptgrey_lib_tag in PtGrey._PtGrey_options:
			self.__ptgrey_class = PtGrey._PtGrey_options[ptgrey_lib_tag](*args, **kwargs)
		else:
			error_msg = 'Invalid ptgrey library requested! Options are: '
			for ptgrey_lib_tag in PtGrey._PtGrey_options:
				error_msg += '\n\t{0}'.format(ptgrey_lib_tag)
			raise Exception(error_msg)

	############### BASE FUNCTIONS ################

	def GetFc2(self):
		'''
		 @brief Get Fc2 python library
		 	Fc2 library contains enums such as fc2.FRAMERATE_30
		'''
		return self.__ptgrey_class.GetFc2()

	def ManualTriggering(self):
		'''
		 @brief Pause progress for user input if manual triggering is on.
		'''
		self.__ptgrey_class.ManualTriggering()

	def CheckManualTriggering(self):
		'''
		 @brief Check if manual triggering is on

		 @return True/False
		'''
		return self.__ptgrey_class.CheckManualTriggering()

	def RetrieveBufferThread(self):
		'''
		 @brief Retrieve buffer frame from ptgrey api.
		 	Sets error_flag = True if any api error
		'''
		self.__ptgrey_class.RetrieveBufferThread()

	def CaptureFrame(self, skip_pin_trig=False, skip_manual_trig=False):
		'''
		 @brief Capture image
		 	Pin triggering is handled if it is set.

		 @param skip_pin_trig (True/False for skipping trigging a new frame on the Ptgrey)
		 @param skip_manual_trig (True/False for skipping manual trig.)

		 @return frame
		'''
		return self.__ptgrey_class.CaptureFrame(skip_pin_trig=skip_pin_trig, skip_manual_trig=skip_manual_trig)

	def AssertFlyCapture2(self):
		'''
		 @brief Assert that the flycapture2 library is available
		'''
		self.__ptgrey_class.AssertFlyCapture2()

	def AssertCameraConnected(self):
		'''
		 @brief Assert camera connected
		'''
		self.__ptgrey_class.AssertCameraConnected()

	def AssertCameraCapturing(self):
		'''
		 @brief Assert camera capturing
		'''
		self.__ptgrey_class.AssertCameraCapturing()

	def CheckCapturing(self):
		'''
		 @brief Check if the camera is capturing frames

		 @return True/False
		'''
		return self.__ptgrey_class.CheckCapturing()

	def ImportFlyCapture2(self):
		'''
		 @brief import pyflycapture2
		 	raies import_flycapture2_error_msg error if flycapture2 library is not available.
		'''
		self.__ptgrey_class.ImportFlyCapture2()

	def ConnectCamera(self):
		'''
		 @brief connect to camera

		 @return camera_info
		'''
		return self.__ptgrey_class.ConnectCamera()

	def DisconnectCamera(self):
		'''
		 @brief disconnect camera
		'''
		self.__ptgrey_class.DisconnectCamera()

	def StartCapture(self):
		'''
		 @brief Start capturing frames
		'''
		self.__ptgrey_class.StartCapture()

	def StopCapture(self):
		'''
		 @brief Stop capturing frames
		'''
		self.__ptgrey_class.StopCapture()

	def RestartCapture(self, n_restart=0, max_restarts=2):
		'''
		 @brief Restart capture due to an api error

		 @param max_restarts (maximum number of restarts before failure raise (default=2))
		'''
		self.__ptgrey_class.RestartCapture(n_restart=n_restart, max_restarts=max_restarts)

	def GetCameraInfo(self):
		'''
		 @brief Get camera info

		 @return camera_info
		'''
		return self.__ptgrey_class.GetCameraInfo()

	def SetTrigger(self, gpio, on_off=True, polarity=None, mode=None):
		'''
		 @brief Set trigger

		 @param gpio (Usually 0, 1, 2, or 3 (2 is tested and working))
		 @param on_off (True/False - turn trigger on/off)
		 @param polarity (True/False - trigger is executed on rising/falling edge - Not updated if == None (default).)
		 @param mode (See documentation - mode 0 is normal trigger mode - Not updated if == None (default).)

		 @return trigger_info
		'''
		return self.__ptgrey_class.SetTrigger(gpio, on_off=on_off, polarity=polarity, mode=mode)

	def SetConfiguration(self, 	num_buffers=None, \
								num_image_notifications=None, \
								min_num_image_notifications=None, \
								grab_timeout=None, \
								grab_mode=None, \
								isoch_bus_speed=None, \
								async_bus_speed=None, 
								bandwidth_allocation=None, \
								register_timeout_retries=None, \
								register_timeout=None):
		'''
		 @brief Set configurations.
		 	<parameter> == None is not configured/updated
		 	All parameters are set to None by default (not updated)

		 @param num_buffers
		 @param num_image_notifications
		 @param min_num_image_notifications
		 @param grab_timeout
		 @param grab_mode
		 @param isoch_bus_speed
		 @param async_bus_speed
		 @param bandwidth_allocation
		 @param register_timeout_retries
		 @param register_timeout

		 @return configurations
		'''
		return self.__ptgrey_class.SetConfiguration(num_buffers=num_buffers, \
													num_image_notifications=num_image_notifications, \
													min_num_image_notifications=min_num_image_notifications, \
													grab_timeout=grab_timeout, \
													grab_mode=grab_mode, \
													isoch_bus_speed=isoch_bus_speed, \
													async_bus_speed=async_bus_speed, \
													bandwidth_allocation=bandwidth_allocation, \
													register_timeout_retries=register_timeout_retries, \
													register_timeout=register_timeout)

	def SetFormat7Configuration(self, mode=None, offset_x=None, offset_y=None, width=None, height=None, pixel_format=None):
		'''
		 @brief Set the format 7 configuration.
		 	<parameter> == None is not configured/updated
		 	All parameters are set to None by default (not updated)


		 @param mode (fc2 enum)
		 @param offset_x
		 @param offset_y
		 @param width
		 @param height
		 @param pixel_format (fc2 enum)

		 @return format7config
		'''
		return self.__ptgrey_class.SetFormat7Configuration(mode=mode, offset_x=offset_x, offset_y=offset_y, width=width, height=height, pixel_format=pixel_format)

	def SetVideoModeAndFrameRate(self, video_mode=None, frame_rate=None):
		'''
		 @brief Set video mode and frame rate
		 	<parameter> == None is not configured/updated
		 	All parameters are set to None by default (not updated)

		 @param video_mode (fc2 enum)
		 @param frame_rate (fc2 enum)

		 @return video_mode, frame_rate (fc2 objects)
		'''
		return self.__ptgrey_class.SetVideoModeAndFrameRate(video_mode=video_mode, frame_rate=frame_rate)

	def SetFrameRate(self, frame_rate=None):
		'''
		 @rief Set frame rate
		 	<parameter> == None is not configured/updated
		 	All parameters are set to None by default (not updated)

		 @param frame_rate (fc2 enum)

		 @return frame_rate_property
		'''
		return self.__ptgrey_class.SetFrameRate(frame_rate=frame_rate)

	def SetGain(self, gain=None, auto=False):
		'''
		 @brief Set gain

		 @param gain (absolute value of gain in dB (f.ex = 10.5 dB))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		return self.__ptgrey_class.SetGain(gain=gain, auto=auto)

	def SetShutter(self, shutter=None, auto=False):
		'''
		 @brief Set shutter

		 @param shutter (absolute value of shutter in ms (f.ex = 20 ms))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		return self.__ptgrey_class.SetShutter(shutter=shutter, auto=auto)

	def SetBrightness(self, brightness=None):
		'''
		 @brief Set shutter

		 @param brightness (absolute value of brigthess in % (f.ex = 0.5 %))
		
		 @return property_settings
		'''
		return self.__ptgrey_class.SetBrightness(brightness=brightness)

	def SetAutoExposure(self, exposure=None, auto=False):
		'''
		 @brief Set auto exposure

		 @param exposure (absolute value of auto exposure in EV (f.ex = -3.5 EV))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		return self.__ptgrey_class.SetAutoExposure(exposure=exposure, auto=auto)

	def SetSharpness(self, sharpness=None, auto=False):
		'''
		 @brief Set sharpness

		 @param sharpness (absolute value of sharpness (f.ex = 1500))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		return self.__ptgrey_class.SetSharpness(sharpness=sharpness, auto=auto)

	def SetGamma(self, gamma=None, auto=False):
		'''
		 @brief Set gamma

		 @param shutter (absolute value of gamma (f.ex = 1.5))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		return self.__ptgrey_class.SetGamma(gamma=gamma, auto=auto)

	def SetWhiteBalance(self, red_channel=None, blue_channel=None, auto=False):
		'''
		 @brief Set gamma

		 @param red_channel (absolute value of red channel (f.ex = 500))
		 @param blue_channel (absolute value of blue channel (f.ex = 850))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		return self.__ptgrey_class.SetWhiteBalance(red_channel=red_channel, blue_channel=blue_channel, auto=auto)

	def __del__(self):
		'''DESTRUCTOR'''
		self.__ptgrey_class.DisconnectCamera()