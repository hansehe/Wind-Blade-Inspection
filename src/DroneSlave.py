'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''
import cv2, timeit, threading, warnings
import numpy as np
'''
 Import bin libraries
'''
from Settings.Settings import Settings
from Settings.Exceptions import DroneVisionError, PtGreyError
from src.bin.tools import GetTimestampedFolder, CheckDir, RemoveDir
from src.DroneVision.DroneVision_src.hardware.PyQtImage import PyQtImage
from src.DroneVision.DroneVision_src.hardware.imageTools import RealTimePlot, WriteImage, GetImage, CheckDisplayAvailable

'''
 Import libraries
'''
from DataBase.DataBase import DataBase
from MasterSlave.Slave import Slave
from DroneVision.DroneVision import DroneVision

'''
 @brief Slave class. Handles the slave module.

 @param settings_inst (Preconfigured settings instance (mainly for testing), used if settings_inst != None, (default=None))
'''
class DroneSlave(Slave, DroneVision, Settings, DataBase):
	def __init__(self, settings_inst=None):
		'''CONSTRUCTOR'''
		Settings.__init__(self, load_initial_settings=False)
		if not(isinstance(settings_inst, type(None))):
			self.ResetSettings(settings_inst.GetRawSettings())
		else:
			self.GetInitialSettings()
		self.__realTimePlot = None
		if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
			if self.GetSettings('REAL_TIME_PLOT', 'use_matplotlib') or not(CheckDisplayAvailable()):
				self.__realTimePlot = RealTimePlot(main_title='Slave', interactive_mode=self.GetSettings('REAL_TIME_PLOT', 'use_interactive_mode'))
			else:
				self.__realTimePlot = PyQtImage(title='Slave')

		Slave.__init__(self, self.GetSettings('TCP'), self)
		DroneVision.__init__(self, False, self.GetSettings(), self.__realTimePlot)
		DataBase.__init__(self, self, self.GetSettings('DATABASE'))
		self.__master_timeout 						= self.GetSettings('BASIC', 'master_timeout')
		self.__new_frame_ready 						= False
		self.__store_db 							= False
		self.__prepared_frame_content 	 			= None
		self.__prepare_lock 	 					= threading.Lock()
		self.__timestamp 		 					= None
		self.__slave_calibrated 		 			= False
		self.__error_flag 							= False
		self.__process_new_frame_flag 				= False
		self.__master_flag 							= None
		self.__force_stereo_vision_calibration		= False
		self.__force_blob_calibration 				= False
		self.__run_calibrate_cv_flag 				= False
		self.__new_traded_frame_flag 				= False 
		self.__traded_frame 						= None
		self.__traded_frame_lock 					= threading.Lock()
		self.__slave_started_flag 					= False
		self.__calibrate_stereopsis_session 			= False # Default
		self.__calibrate_blob_scale_detector_session 	= False # Default

	def InitializeSlave(self):
		'''
		 @brief Main initializing function for the slave. 
		 	Locks the slave in a infinite loop.
		'''
		self.Connect() 	#Connect to master
		self.WaitTimestampFromMaster()
		self.SetTimestampedFolder(self.__timestamp)
		self.InitDroneVision()
		self.SetDataBaseOutputFolder(self.GetCameraOutputFolder())
		if isinstance(self.__realTimePlot, RealTimePlot):
			self.__realTimePlot.SetSaveFigFolder(self.GetDatabaseOutputFolder() + self.GetSettings('REAL_TIME_PLOT', 'save_fig_folder'))

	def RunSlave(self):
		'''
		 @brief Run slave indefinitely. Master controls getFrame/stop/restart.
		'''
		while not(self.GetTerminate()):
			if self.__run_calibrate_cv_flag:
				self.__run_calibrate_cv_flag = False
				self.RunCalibrateCV()
				self.SetDatabaseTableName(self.__timestamp)
				self.WaitForMasterFlag() # Wait for master signal to start
				if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
					self.__realTimePlot(reset=True)
				self.__slave_started_flag 	= True # Flag that slave has been configured and started
				self.ResetMasterTimeout()
			if self.__process_new_frame_flag:
				self.__process_new_frame_flag = False 
				self.ProcessNewFrame()
				self.ResetMasterTimeout()
			if self.__store_db:
				self.__store_db = False
				with self.__prepare_lock:
					frame_content = self.__prepared_frame_content

				self.RecordData(record_frames=False, insert_to_database=False, print_progress=True)

				#--------- SHOW RESULTS REALTIME --------#
				if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
					plot_frames = [('original_right', frame_content[0]), ('original_sl_right', frame_content[1])]
					self.__realTimePlot(plot_frames)
				#----------------------------------------#
				self.ResetMasterTimeout()
			self.CheckMasterTimeout()

		self.PrintFinished()
		self.GetFinishRequest()
		self.CloseSlave()

	def GetFinishRequest(self):
		'''
		 @brief Get finish request from master to slave
		'''
		while self.CheckConnected() and not(self.CheckErrorFlag()): # Wait for master to disconnect slave
			pass

	def ResetMasterTimeout(self):
		'''
		 @brief Reset the master timeout
		'''
		self.__master_delay = timeit.default_timer()

	def CheckMasterTimeout(self):
		'''
		 @brief Check if master hasn't given any requests during a given timeout.
		 	Raise error if master times out
		'''
		# Check settings which assert that any specifics of user control during operation doesn't block the program.
		if self.GetSettings('USER_INPUT', 'automatic_mode') and self.__slave_started_flag:
			if timeit.default_timer() - self.__master_delay > self.__master_timeout:
				raise Exception('Master timed out!')


	def CheckRunSlaveCalibration(self, calibrate_stereopsis=False, calibrate_blob_scale_detector=False):
		'''
		 @brief Check if a new calibration session should be started.
		 
		 @param calibrate_stereopsis (Set calibrate_stereopsis to True for starting a new stereopsis calibration session (default=False))
		 @param calibrate_blob_scale_detector (Set calibrate_blob_scale_detector to True for starting a new blob scale detector calibration session (default=False))
		'''
		n_saved_calib_frame_sets = 0
		if calibrate_stereopsis:
			calib_folder = self.GetSettings('CALIB', 'calib_img_folder_right_cam')
			n_saved_calib_frame_sets += self.RunSlaveCalibration(calib_folder, True)
		if calibrate_blob_scale_detector:
			calib_folder = self.GetSettings('BLOB_SCALE', 'scale_calib_folder')
			n_saved_calib_frame_sets += self.RunSlaveCalibration(calib_folder, False)
		if (calibrate_stereopsis or calibrate_blob_scale_detector) and not(self.GetSettings('BASIC', 'reset_calibration')):
			self.__force_stereo_vision_calibration 	= self.WaitForMasterFlag()
			self.__force_blob_calibration 			= self.WaitForMasterFlag()
		if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
			self.__realTimePlot(reset=True)

	def RunSlaveCalibration(self, calib_folder, stereo_or_blob_calib, calib_filename='calib_frame'):
		'''
		 @brief Run slave calibration.
		 	Manually trig new images of a chessboard to calibrate the stereopsis system.

		 @param calib_folder (folder where the captured calibration frames are stored)
		 @param stereo_or_blob_calib (True for stereopsis calibration, False for blob scale calibration)
		 @param calib_filename (Basic filename of save frames)

		 @return n_saved_calib_frame_sets
		'''
		first_image_set = True
		if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
			self.__realTimePlot(reset=True)
		self.PrintCalibrationSession(calib_folder, stereo_or_blob_calib, calib_filename)
		if stereo_or_blob_calib:
			get_normal_frame_only = True
		else:
			normal_folder 	= calib_folder + 'normal/'
			sl_folder 		= calib_folder + 'sl/'
			get_normal_frame_only = False # blob calibration demands sl frames
		frame_n = 0
		while True:
			self.__slave_calibrated = False
			continue_answer = self.WaitForMasterFlag()
			if not(continue_answer):
				break # Master commanded slave to break
			while not(self.__process_new_frame_flag):
				pass # Wait for master to flag new frame capturing.
			try:
				frame, sl_frame = self.GetRawFrames(get_normal_frame_only=get_normal_frame_only)
				self.__process_new_frame_flag 	= False
				self.__slave_calibrated 		= True # Flag captured frames to master
			except PtGreyError, err:
				warnings.simplefilter('always')
				warnings.warn(str(err), Warning)
				warnings.simplefilter('default')
				self.RestartCamera()
				continue

			#-------- SHOW RESULTS REALTIME ---------#
			if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
				if get_normal_frame_only:
					title 		= 'stereopsis_calibration_session'
					plot_frames = [('frame', frame)]
				else:
					title 		= 'blob_calibration_session'
					plot_frames = [('frame', frame), ('sl_frame', sl_frame)]
				self.__realTimePlot(plot_frames, title)
			#----------------------------------------#

			answer = self.WaitForMasterFlag()
			if answer:
				if first_image_set:
					first_image_set = False
					if get_normal_frame_only:
						RemoveDir(calib_folder)
						CheckDir(calib_folder)
					else:
						RemoveDir(normal_folder)
						RemoveDir(sl_folder)
						CheckDir(normal_folder)
						CheckDir(sl_folder)
				print 'Slave is saving calibration frames [{0}]..'.format(frame_n+1)
				if get_normal_frame_only:
					WriteImage(frame, calib_folder + calib_filename + '_' + str(frame_n))
				else:
					WriteImage(frame, normal_folder + calib_filename + '_' + str(frame_n))
					WriteImage(sl_frame, sl_folder + calib_filename + '_' + str(frame_n))
				frame_n += 1
			else:
				print 'Slave did not save calibration frames..'
		self.__slave_calibrated 		= False
		self.__process_new_frame_flag 	= False
		if get_normal_frame_only:
			if frame_n > 0:
				RemoveDir(self.GetSettings('CALIB', 'calib_img_folder_left_cam'))
				CheckDir(self.GetSettings('CALIB', 'calib_img_folder_left_cam'))
			finished_trade = None
			n_traded_frame = 0
			while not(isinstance(finished_trade, bool)) and not(self.CheckErrorFlag()):
				finished_trade = self.GetMasterFlag() # Master sends a True/False flag for finished trading frames
				if self.__new_traded_frame_flag:
					with self.__traded_frame_lock:
						WriteImage(self.__traded_frame, self.GetSettings('CALIB', 'calib_img_folder_left_cam') + calib_filename + '_' + str(n_traded_frame))
						self.__new_traded_frame_flag = False
						n_traded_frame += 1
						print 'Traded frame {0}/{1} with master..'.format(n_traded_frame, frame_n)
			self.SetFlagFromMaster(None) # Reset the master flag
		return frame_n

	def WaitForMasterFlag(self):
		'''
		 @brief Wait for master flag

		 @return flag
		'''
		flag = None
		while not(isinstance(flag, bool)) and not(self.CheckErrorFlag()):
			flag = self.GetMasterFlag() # Master sends a True/False flag for saving calibration frames
		self.SetFlagFromMaster(None) # Reset the master flag
		return flag

	def SetFlagFromMaster(self, flag):
		'''
		 @brief Set the master flag.
		 	Simple communication from master
		'''
		self.__master_flag = flag

	def GetMasterFlag(self):
		'''
		 @brief Get the master flag.
		'''
		return self.__master_flag

	def SetProcessNewFrameFlag(self):
		'''
		 @brief Set process new frame flah
		'''
		self.__process_new_frame_flag = True

	def ProcessNewFrame(self):
		'''
		 @brief Prepare new frame to be requested by master.

		 @return frame 
		'''
		self.__error_flag = False
		if not(self.CheckDroneVisionFinished()):
			try:
				original_frame, original_sl_frame = self.GetRawFrames()
				if self.GetSettings('DATABASE', 'store_frames_as_video') or self.GetSettings('DATABASE', 'store_frames_as_images'): # Store frames here to relieve memory. The frames are deleted as soon as possible.
					self.SetProcessFrame('original_right', original_frame)
					self.SetProcessFrame('original_sl_right', original_sl_frame)
					self.RecordData(record_frames=True, insert_to_database=False)
				tmp_frame_content = self.GetProcessedFrame(original_frame=original_frame, original_sl_frame=original_sl_frame)
			except (DroneVisionError, PtGreyError), err:
				self.__error_flag = True
				warnings.simplefilter('always')
				warnings.warn(str(err), Warning)
				warnings.simplefilter('default')
				if not(isinstance(err, PtGreyError)): # PtGrey did not fail, store captured frames.
					tmp_frame_content = (original_frame, original_sl_frame)
				else:
					return # PtGrey error - dominant error, so return.
			with self.__prepare_lock:
				self.__prepared_frame_content = tmp_frame_content
			if self.__error_flag:
				self.SetStoreDBFlag() # Store failed frames to database.
			else:
				self.__new_frame_ready = True # Frames will be stored after they are sent to master.

	def GetFramePayload(self):
		'''
		 @brief Get Frame, with frame information, as a dictionary.
		  Used by Slave instance to send a getFrame response to master.

		 @return content Dictionary of frame with information.
		'''
		frame_content = None
		if self.__new_frame_ready:
			with self.__prepare_lock:
				frame_content = self.__prepared_frame_content
			self.__new_frame_ready 	= False
		return frame_content, self.__error_flag

	def SetStoreDBFlag(self):
		'''
		 @brief set store to database flag to true
		'''
		self.__store_db = True

	def GetOriginalFramePayload(self, filename, sl_filename=None):
		'''
		 @brief Get original frame from a filename

		 @param filename
		 @param sl_filename (default=None)
		'''
		try:
			original_frame = GetImage(filename)
			if sl_filename != None:
				original_sl_frame = GetImage(sl_filename)
				frame_content = (original_frame, original_sl_frame)
			else:
				frame_content = (original_frame)
		except Exception, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
			return None, True
		return frame_content, False

	def SetNewTradedFrame(self, traded_frame):
		'''
		 @brief Set new traded frame
		'''
		with self.__traded_frame_lock:
			self.__traded_frame = traded_frame
			self.__new_traded_frame_flag = True

	def WaitTimestampFromMaster(self):
		timeout = timeit.default_timer()
		while self.__timestamp == None:
			if (timeit.default_timer() - timeout) > self.GetSettings('TCP', 'tcp_timeout'):
				self.Disconnect()
				raise Exception('Timeout - no timestamp message from master')

	def CalibrateCV(self, calibrate_stereopsis_session, calibrate_blob_scale_detector_session):
		'''
		 @brief Calibrate CV.

		 @param calibrate_stereopsis_session (see droneMaster)
		 @param calibrate_blob_scale_detector_session (see droneMaster)
		'''
		self.__calibrate_stereopsis_session 			= calibrate_stereopsis_session
		self.__calibrate_blob_scale_detector_session 	= calibrate_blob_scale_detector_session
		self.__run_calibrate_cv_flag = True

	def RunCalibrateCV(self):
		'''
		 @brief Run calibrate CV.
		'''
		self.CheckRunSlaveCalibration(self.__calibrate_stereopsis_session, self.__calibrate_blob_scale_detector_session)
		self.CalibratePointDetection(force_calibration=self.__force_stereo_vision_calibration, force_blob_calibration=self.__force_blob_calibration)
		self.__slave_calibrated = True

	def GetSlaveReady(self):
		'''
		 @brief Check if slave is ready after camera calibration and intialization of the droneVision class.

		 @return True/False
		'''
		ready = False
		if self.__slave_calibrated and self.GetDroneVisionReady():
			ready = True
		return ready

	def SetTimestamp(self, timestamp):
		'''
		 @brief Set timestamp for this session

		 @param timestamp
		'''
		self.__timestamp = timestamp

	def SetTimestampedFolder(self, timestamp):
		'''
		 @brief Set timestamped folder for this session.

		 @param timestamp String
		'''
		self.ChangeSetting('DATABASE', 'output_folder', str(GetTimestampedFolder(timestamp, self.GetSettings('DATABASE', 'output_folder'), self.GetSettings('DATABASE', 'sub_output_folder'))))
		
	def CloseSlave(self):
		'''
		 @brief Close slave safely
		'''
		Slave.__del__(self)
		DataBase.__del__(self)

	def __del__(self):
		'''DESTRUCTOR'''
		self.CloseSlave()
