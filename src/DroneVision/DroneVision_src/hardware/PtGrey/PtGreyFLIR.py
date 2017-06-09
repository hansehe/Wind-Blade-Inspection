'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

from src.bin.tools import RunThread
from Settings.Exceptions import PtGreyError
from ..PinControl import PinControl

import numpy as np
from getpass import getpass
import warnings, time

'''
 @brief Class for handling the Point Grey camera using python wrapper PyCapture2, developed by FLIR (official).
 	See HOWTO.txt on how to install PyCapture2.
 	See PyCapture2 document for information.

 @param mc_trigger_pin (trigger pin on the micro controler. Set to -1 for no trig (default=-1))
 @param ptgrey_trigger_pin (trigger pin on the PtGrey camera. Set to -1 for no trig (usually 0, 1, 2, or 3. 2 is tested and working) (default=2))
 @param manual_triggering (True/False - Pause script before trig to manually set camera in place before triggering (default=False).)
 @param capture_timeout (timeout for capturing a frame - Set negative for infinite waiting time)

 !!!!! NOT FINISHED !!!!!!
'''
class PtGreyFLIR(object):
	_PtGrey_lib_tag = 'FLIR' # Static tag for checking which library this is

	def __init__(self, mc_trigger_pin=-1, ptgrey_trigger_pin=2, manual_triggering=False, capture_timeout=2.0):
		'''CONSTRUCTOR'''
		self.__mc_trigger_pin 		= PinControl(mc_trigger_pin)
		self.__ptgrey_trigger_pin 	= ptgrey_trigger_pin
		self.__manual_triggering 	= manual_triggering
		self.__capture_timeout 		= capture_timeout
		self.__fc2 					= None
		self.__fc2_camera 			= None
		self.__camera_connected 	= False
		self.__capturing 			= False
		self.__error_flag 			= False

		self.ImportFlyCapture2()
		print self.ConnectCamera()
		if self.__mc_trigger_pin.GetPin() >= 0 and self.__ptgrey_trigger_pin >= 0:
			self.SetTrigger(self.__ptgrey_trigger_pin, on_off=True, polarity=False, mode=0)

	def GetFc2(self):
		'''
		 @brief Get Fc2 python library
		 	Fc2 library contains enums such as fc2.FRAMERATE_30
		'''
		return self.__fc2

	def ManualTriggering(self):
		'''
		 @brief Pause progress for user input if manual triggering is on.
		'''
		if self.__manual_triggering:
			getpass('Press enter to trigger camera..')

	def CheckManualTriggering(self):
		'''
		 @brief Check if manual triggering is on

		 @return True/False
		'''
		return self.__manual_triggering

	def RetrieveBufferThread(self):
		'''
		 @brief Retrieve buffer frame from ptgrey api.
		 	Sets error_flag = True if any api error
		'''
		self.__error_flag = False
		try:
			self.__fc2_camera.retrieveBuffer(self.__frame_object)
		except Exception, err:
			self.__error_flag = True
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')

	def ToggleTriggerPin(self, toggle_delay=0.0):
		'''
		 @brief Toggle camera trigger pin to make camera capture frame

		 @param toggle_delay (float)
		'''
		if self.__mc_trigger_pin.GetPin() >= 0:
			self.__mc_trigger_pin.TogglePin()
			if toggle_delay > 0:
				time.sleep(toggle_delay)

	def CaptureFrame(self, skip_pin_trig=False, skip_manual_trig=False):
		'''
		 @brief Capture image
		 	Pin triggering is handled if it is set.

		 @param skip_pin_trig (True/False for skipping trigging a new frame on the Ptgrey)
		 @param skip_manual_trig (True/False for skipping manual trig.)

		 @return frame
		'''
		self.AssertCameraCapturing()
		if not(skip_manual_trig):
			self.ManualTriggering()
		if self.__mc_trigger_pin.GetPin() >= 0 and not(skip_pin_trig):
			self.__mc_trigger_pin.TogglePin()
		t = RunThread(self.RetrieveBufferThread)
		if self.__capture_timeout >= 0:
			t.join(self.__capture_timeout)
		else:
			t.join()
		if t.isAlive():
			raise PtGreyError('timeout_capturing_frame_error_msg')
		elif self.__error_flag:
			self.__error_flag = False
			raise PtGreyError('failed_capturing_frame_error_msg')
		if self.CheckManualTriggering():
			time.sleep(1.0) # Sleep camera if manual trig to make it dump the image buffer.
		return np.array(self.__frame_object)

	def AssertFlyCapture2(self):
		'''
		 @brief Assert that the flycapture2 library is available
		'''
		if self.__fc2 == None:
			raise PtGreyError('import_flycapture2_error_msg')

	def AssertCameraConnected(self):
		'''
		 @brief Assert camera connected
		'''
		if self.__fc2_camera == None or not(self.__camera_connected):
			raise PtGreyError('camera_not_connected_error_msg')

	def AssertCameraCapturing(self):
		'''
		 @brief Assert camera capturing
		'''
		if not(self.CheckCapturing()):
			raise PtGreyError('camera_not_capturing_error_msg')

	def CheckCapturing(self):
		'''
		 @brief Check if the camera is capturing frames

		 @return True/False
		'''
		return self.__capturing

	def ImportFlyCapture2(self):
		'''
		 @brief import pyflycapture2
		 	raies import_flycapture2_error_msg error if flycapture2 library is not available.
		'''
		try:
			import PyCapture2 as fc2
			self.__fc2 = fc2
		except ImportError:
			raise PtGreyError('import_fc2_error_msg')

	def ConnectCamera(self):
		'''
		 @brief connect to camera

		 @return camera_info
		'''
		# Ensure sufficient cameras are found
		bus = self.__fc2.BusManager()
		numCams = bus.getNumOfCameras()
		self.__fc2_camera = self.__fc2.Context()
		if bus.getNumOfCameras() == 0:
			raise PtGreyError('camera_not_connected_error_msg')
		self.__fc2_camera = self.__fc2.Camera()
		self.__fc2_camera.connect(bus.getCameraFromIndex(0))
		self.__frame_object = self.__fc2.Image()

		# Power on the Camera
		cameraPower = 0x610
		powerVal = 0x80000000

		self.__fc2_camera.writeRegister(cameraPower, powerVal)

		# Waiting for camera to power up
		retries = 10
		timeToSleep = 0.1	#seconds
		for i in range(retries):
			sleep(timeToSleep)
			try:
				regVal = self.__fc2_camera.readRegister(cameraPower)
			except self.__fc2.Fc2error:	# Camera might not respond to register reads during powerup.
				pass
			awake = True
			if regVal == powerVal:
				break
			awake = False
		if not awake:
			print "Could not wake Camera. Exiting..."
			raise PtGreyError('camera_not_connected_error_msg')

		self.__camera_connected = True
		return self.GetCameraInfo()

	def DisconnectCamera(self):
		'''
		 @brief disconnect camera
		'''
		self.AssertCameraConnected()
		if self.__capturing:
			self.StopCapture()
		try:
			self.__fc2_camera.disconnect()
		except Exception, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
		self.__camera_connected = False

	def StartCapture(self):
		'''
		 @brief Start capturing frames
		'''
		self.AssertCameraConnected()
		self.__fc2_camera.startCapture()
		self.__capturing = True

	def StopCapture(self):
		'''
		 @brief Stop capturing frames
		'''
		self.AssertCameraConnected()
		try:
			self.__fc2_camera.stopCapture()
		except Exception, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
		self.__capturing = False

	def RestartCapture(self, n_restart=0, max_restarts=2):
		'''
		 @brief Restart capture due to an api error
		 	Please run StartCapture() after restarting

		 @param max_restarts (maximum number of restarts before failure raise (default=2))
		'''
		self.DisconnectCamera()
		try:
			self.ConnectCamera()
			if self.__mc_trigger_pin.GetPin() >= 0 and self.__ptgrey_trigger_pin >= 0:
				self.SetTrigger(self.__ptgrey_trigger_pin, on_off=False, polarity=False, mode=0)
				self.SetTrigger(self.__ptgrey_trigger_pin, on_off=True, polarity=False, mode=0)
		except:
			if n_restart < max_restarts:
				self.RestartCapture(n_restart=n_restart+1)
			else:
				raise

	def GetCameraInfo(self):
		'''
		 @brief Get camera info

		 @return camera_info
		'''
		self.AssertCameraConnected()
		return self.__fc2_camera.getCameraInfo()

	def SetTrigger(self, gpio, on_off=True, polarity=None, mode=None):
		'''
		 @brief Set trigger

		 @param gpio (Usually 0, 1, 2, or 3 (2 is tested and working))
		 @param on_off (True/False - turn trigger on/off)
		 @param polarity (True/False - trigger is executed on rising/falling edge - Not updated if == None (default).)
		 @param mode (See documentation - mode 0 is normal trigger mode - Not updated if == None (default).)

		 @return trigger_info
		'''
		trigger_info = self.__fc2_camera.getTriggerMode()
		trigger_info['source'] 		= gpio
		trigger_info['on_off'] 		= int(on_off)
		if not(polarity == None):
			trigger_info['polarity'] = int(polarity)
		if not(mode == None):
			trigger_info['mode'] = mode
		self.__fc2_camera.setTriggerMode(**trigger_info)
		return trigger_info

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
		self.AssertCameraConnected()
		configurations = self.__fc2_camera.getConfiguration()
		if not(num_buffers == None):
			configurations['num_buffers'] = num_buffers
		if not(num_image_notifications == None):
			configurations['num_image_notifications'] = num_image_notifications
		if not(min_num_image_notifications == None):
			configurations['min_num_image_notifications'] = min_num_image_notifications
		if not(grab_timeout == None):
			configurations['grab_timeout'] = grab_timeout
		if not(grab_mode == None):
			configurations['grab_mode'] = grab_mode
		if not(isoch_bus_speed == None):
			configurations['isoch_bus_speed'] = isoch_bus_speed
		if not(async_bus_speed == None):
			configurations['async_bus_speed'] = async_bus_speed
		if not(bandwidth_allocation == None):
			configurations['bandwidth_allocation'] = bandwidth_allocation
		if not(register_timeout_retries == None):
			configurations['register_timeout_retries'] = register_timeout_retries
		if not(register_timeout == None):
			configurations['register_timeout'] = register_timeout
		if not(self.__capturing):
			self.__fc2_camera.setConfiguration(**configurations)
		else:
			warnings.warn('Cannot set configuration while capturing.', UserWarning)
		return configurations

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
		self.AssertCameraConnected()
		format7config = self.__fc2_camera.getFormat7Info()
		if not(mode == None):
			format7config['mode'] = mode
		if not(offset_x == None):
			format7config['offset_x'] = offset_x
		if not(offset_y == None):
			format7config['offset_y'] = offset_y
		if not(width == None):
			format7config['width'] = width
		if not(height == None):
			format7config['height'] = height
		if not(pixel_format == None):
			format7config['pixel_format'] = pixel_format
		if not(self.__capturing):
			# Configure camera format7 settings
			fmt7imgSet = PyCapture2.Format7ImageSettings(**format7config)
			fmt7pktInf, isValid = c.validateFormat7Settings(fmt7imgSet)
			if not isValid:
				warnings.warn("Format7 settings are not valid!", UserWarning)
			self.__fc2_camera.setFormat7ConfigurationPacket(fmt7pktInf.recommendedBytesPerPacket, fmt7imgSet)
		else:
			warnings.warn('Cannot set format7 configuration while capturing.', UserWarning)
		return format7config

	def SetVideoModeAndFrameRate(self, video_mode=None, frame_rate=None):
		'''
		 @brief Set video mode and frame rate
		 	<parameter> == None is not configured/updated
		 	All parameters are set to None by default (not updated)

		 @param video_mode (fc2 enum)
		 @param frame_rate (fc2 enum)

		 @return video_mode, frame_rate (fc2 objects)
		'''
		self.AssertCameraConnected()
		pre_vid_mode, pre_frame_rate = self.__fc2_camera.getVideoModeAndFrameRate()
		if not(video_mode == None):
			pre_vid_mode = video_mode
		if not(frame_rate == None):
			pre_frame_rate = frame_rate
		if not(self.__capturing):
			self.__fc2_camera.setVideoModeAndFrameRate(pre_vid_mode, pre_frame_rate)
		else:
			warnings.warn('Cannot set video mode and frame rate while capturing.', UserWarning)
		return self.__fc2_camera.getVideoModeAndFrameRate()

	def SetFrameRate(self, frame_rate=None):
		'''
		 @rief Set frame rate
		 	<parameter> == None is not configured/updated
		 	All parameters are set to None by default (not updated)

		 @param frame_rate (fc2 enum)

		 @return frame_rate_property
		'''
		frame_rate_property = self.__fc2_camera.getProperty(self.__fc2.FRAME_RATE)
		if frame_rate != None:
			frame_rate_property['abs_value'] = float(frame_rate)
		self.__fc2_camera.setProperty(**frame_rate_property)
		return frame_rate_property

	def SetGain(self, gain=None, auto=False):
		'''
		 @brief Set gain

		 @param gain (absolute value of gain in dB (f.ex = 10.5 dB))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		self.AssertCameraConnected()
		prop = self.__fc2_camera.getProperty(self.__fc2.GAIN)
		prop['auto_manual_mode'] 	= int(auto)
		prop['abs_control'] 		= int(True)
		if gain != None:
			prop['abs_value'] 		= gain
		self.__fc2_camera.setProperty(**prop)
		return prop

	def SetShutter(self, shutter=None, auto=False):
		'''
		 @brief Set shutter

		 @param shutter (absolute value of shutter in ms (f.ex = 20 ms))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		self.AssertCameraConnected()
		prop = self.__fc2_camera.getProperty(self.__fc2.SHUTTER)
		prop['on_off'] 				= int(True)
		prop['auto_manual_mode'] 	= int(auto)
		prop['abs_control'] 		= int(True)
		if shutter != None:
			prop['abs_value'] 		= shutter
		self.__fc2_camera.setProperty(**prop)
		return prop

	def SetBrightness(self, brightness=None):
		'''
		 @brief Set shutter

		 @param brightness (absolute value of brigthess in % (f.ex = 0.5 %))
		
		 @return property_settings
		'''
		self.AssertCameraConnected()
		prop = self.__fc2_camera.getProperty(self.__fc2.BRIGHTNESS)
		prop['abs_control'] 		= int(True)
		if brightness != None:
			prop['abs_value'] 		= brightness
		self.__fc2_camera.setProperty(**prop)
		return prop

	def SetAutoExposure(self, exposure=None, auto=False):
		'''
		 @brief Set auto exposure

		 @param exposure (absolute value of auto exposure in EV (f.ex = -3.5 EV))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		self.AssertCameraConnected()
		prop = self.__fc2_camera.getProperty(self.__fc2.AUTO_EXPOSURE)
		prop['on_off'] 				= int(True)
		prop['auto_manual_mode'] 	= int(auto)
		prop['abs_control'] 		= int(True)
		if exposure != None:
			prop['abs_value'] 		= exposure
		self.__fc2_camera.setProperty(**prop)
		return prop

	def SetSharpness(self, sharpness=None, auto=False):
		'''
		 @brief Set sharpness

		 @param sharpness (absolute value of sharpness (f.ex = 1500))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		self.AssertCameraConnected()
		prop = self.__fc2_camera.getProperty(self.__fc2.SHARPNESS)
		prop['on_off'] 				= int(True)
		prop['auto_manual_mode'] 	= int(auto)
		prop['abs_control'] 		= int(True)
		if sharpness != None:
			prop['value_a'] 		= sharpness
		self.__fc2_camera.setProperty(**prop)
		return prop

	def SetGamma(self, gamma=None, auto=False):
		'''
		 @brief Set gamma

		 @param shutter (absolute value of gamma (f.ex = 1.5))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		self.AssertCameraConnected()
		prop = self.__fc2_camera.getProperty(self.__fc2.GAMMA)
		prop['on_off'] 				= int(True)
		prop['auto_manual_mode'] 	= int(auto)
		prop['abs_control'] 		= int(True)
		if gamma != None:
			prop['abs_value'] 		= gamma
		self.__fc2_camera.setProperty(**prop)
		return prop

	def SetWhiteBalance(self, red_channel=None, blue_channel=None, auto=False):
		'''
		 @brief Set gamma

		 @param red_channel (absolute value of red channel (f.ex = 500))
		 @param blue_channel (absolute value of blue channel (f.ex = 850))
		 @param auto (True/False - override manual settings if True)
		
		 @return property_settings
		'''
		self.AssertCameraConnected()
		prop = self.__fc2_camera.getProperty(self.__fc2.GAMMA)
		prop['on_off'] 				= int(True)
		prop['auto_manual_mode'] 	= int(auto)
		prop['abs_control'] 		= int(True)
		if red_channel != None and blue_channel != None:
			prop['value_a'] 			= red_channel
			prop['value_b'] 			= blue_channel
		self.__fc2_camera.setProperty(**prop)
		return prop

	def __del__(self):
		'''DESTRUCTOR'''
		self.DisconnectCamera()




