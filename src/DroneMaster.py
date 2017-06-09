'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''
import numpy as np
import timeit, time, warnings
from getpass import getpass
'''
 Import bin libraries
'''
from src.bin.tools import RunThread, GetTimestamp, GetTimestampedFolder, CheckDir, RemoveDir
from Settings.Settings import Settings
from Settings.Exceptions import DroneVisionError, PtGreyError
from src.DroneVision.DroneVision_src.hardware.PyQtImage import PyQtImage
from src.DroneVision.DroneVision_src.hardware.imageTools import RealTimePlot, WriteImage, GetImage, CheckDisplayAvailable
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape

'''
 Import src libraries
'''
from DataBase.DataBase import DataBase
from MasterSlave.Master import Master
from DroneVision.DroneVision import DroneVision

'''
 User input
'''
from src.bin.UserInput.UserInput import UserInput

'''
 @brief Master class. Handles the master module.

 @param settings_inst (Preconfigured settings instance (mainly for testing), used if settings_inst != None, (default=None))
 @param calibrate_stereopsis_session (Set calibrate_stereopsis_session to True for starting a new stereopsis calibration session (default=False))
 @param calibrate_blob_scale_detector_session (Set calibrate_blob_scale_detector_session to True for starting a new blob scale detector calibration session (default=False))
'''
class DroneMaster(Master, DroneVision, Settings, DataBase, UserInput):
	def __init__(self, settings_inst=None, calibrate_stereopsis_session=False, calibrate_blob_scale_detector_session=False):
		'''CONSTRUCTOR'''
		Settings.__init__(self, load_initial_settings=False)
		if not(isinstance(settings_inst, type(None))):
			self.ResetSettings(settings_inst.GetRawSettings())
		else:
			self.GetInitialSettings()
		self.__realTimePlot = None
		if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
			if self.GetSettings('REAL_TIME_PLOT', 'use_matplotlib') or not(CheckDisplayAvailable()):
				self.__realTimePlot = RealTimePlot(main_title='Master', interactive_mode=self.GetSettings('REAL_TIME_PLOT', 'use_interactive_mode'))
			else:
				self.__realTimePlot = PyQtImage(title='Master')

		Master.__init__(self, self.GetSettings('TCP'))
		DroneVision.__init__(self, True, self.GetSettings(), self.__realTimePlot)
		DataBase.__init__(self, self, self.GetSettings('DATABASE'))
		UserInput.__init__(self, self.GetSettings('USER_INPUT'))
		self.__frame_un_r_shape							= None
		self.__keypoints_r 								= None
		self.__descriptors_r 							= None
		self.__req_success 								= False
		self.__req_error 								= False
		self.__force_stereo_vision_calibration			= False
		self.__force_blob_calibration 					= False
		self.__calibrate_stereopsis_session 			= calibrate_stereopsis_session
		self.__calibrate_blob_scale_detector_session 	= calibrate_blob_scale_detector_session

	def InitializeMaster(self):
		'''
		 @brief Main initializing function for the master. 
		 	Locks the master in a infinite loop.
		'''
		self.SetTimestamp(GetTimestamp())
		self.Connect() 	#Connect to slave
		self.SetTimestampedFolder(self.__timestamp)
		self.RequestSetTimestamp(self.__timestamp)
		self.InitDroneVision()
		self.SetDataBaseOutputFolder(self.GetCameraOutputFolder())
		if isinstance(self.__realTimePlot, RealTimePlot):
			self.__realTimePlot.SetSaveFigFolder(self.GetDatabaseOutputFolder() + self.GetSettings('REAL_TIME_PLOT', 'save_fig_folder'))
		self.RequestCVCalibration(self.__calibrate_stereopsis_session, self.__calibrate_blob_scale_detector_session)
		self.CheckRunMasterCalibration(self.__calibrate_stereopsis_session, self.__calibrate_blob_scale_detector_session)
		self.CalibratePointDetection(force_calibration=self.__force_stereo_vision_calibration, force_blob_calibration=self.__force_blob_calibration)
		self.WaitSlaveReady()
		self.SetDatabaseTableName(self.__timestamp, master=True, wait_for_user=not(self.GetSettings('USER_INPUT', 'automatic_mode')))
		self.SendFlagToSlave(True) # Send flag to slave commanding it to continue

	def RunMaster(self):
		'''
		 @brief Run master indefinitely.
		 	Implement computer vision, manouvering and gimbal control here.
		'''
		store_to_db  			= self.GetSettings('DATABASE', 'store_process_data')
		store_frames 			= self.GetSettings('DATABASE', 'store_frames_as_video') or self.GetSettings('DATABASE', 'store_frames_as_images')
		store_drawn_frames 		= self.GetSettings('DATABASE', 'store_drawn_frames')
		draw_heading 			= self.GetSettings('DATABASE', 'draw_heading')
		draw_matches 			= self.GetSettings('DATABASE', 'draw_matches')
		draw_hough_lines 		= self.GetSettings('DATABASE', 'draw_hough_lines')
		draw_detected_points 	= self.GetSettings('DATABASE', 'draw_detected_points')
		print_3D_points 		= self.GetSettings('DATABASE', 'print_3D_points')
		if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
			self.__realTimePlot(reset=True)
		self.StartAutoHandleUserInput()
		self.ResetTermination()
		while not(self.CheckFinished()):
			##########################################

			#----------- COMPUTER VISION ------------#
			points_error, boundary_error, heading_error, stereo_error, heading_distance, heading_angle, points3D, frame_un_l, delta_frame_l, hough_frame, matches_frame = self.ProcessCV(draw_heading=draw_heading, draw_hough_lines=draw_hough_lines, draw_detected_points=draw_detected_points, draw_matches=draw_matches)
			if isinstance(points_error, PtGreyError): # Dominant error - continue with next frame
				continue
			if stereo_error == None and points_error == None:
				points3D_m 		= self.Points3DToMatrix(points3D)
				average_point3D = np.mean(points3D_m, axis=1)
				std_points3D 	= np.std(points3D_m, axis=1)
			#----------------------------------------#

			#----------- GIMBAL CONTROL -------------#
			# TODO (implement)
			#----------------------------------------#

			#----------- DRONE MANOUVERING ----------#
			# TODO (implement)
			#----------------------------------------#

			#----------- STORE IN DATABASE ----------#
			if points_error == None:
				if stereo_error == None:
					self.SetProcessData('X_average', average_point3D[0,0])
					self.SetProcessData('Y_average', average_point3D[1,0])
					self.SetProcessData('Z_average', average_point3D[2,0])
					self.SetProcessData('Z_std', std_points3D[2,0])
				if heading_error == None:
					self.SetProcessData('rho', heading_distance)
					self.SetProcessData('theta', heading_angle)
				if store_frames and store_drawn_frames:
					if draw_heading:
						self.SetProcessFrame('heading', frame_un_l)
					if draw_matches and not(isinstance(stereo_error, DroneVisionError)):
						self.SetProcessFrame('matches', matches_frame)
					if draw_hough_lines and not(isinstance(boundary_error, DroneVisionError)):
						self.SetProcessFrame('hough_lines', hough_frame)
					if draw_detected_points:
						self.SetProcessFrame('points', delta_frame_l)

			if not(print_3D_points) or stereo_error != None or points_error != None:
				points3D = []
			self.RecordData(record_frames=store_frames, insert_to_database=store_to_db, print_progress=True, points3D=points3D)
			#----------------------------------------#

			#-------- SHOW RESULTS REALTIME ---------#
			if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
				if not(isinstance(points_error, DroneVisionError)):
					plot_frames = []
					if draw_heading:
						plot_frames.append(('heading', frame_un_l))
					if draw_matches and not(isinstance(stereo_error, DroneVisionError)):
						plot_frames.append(('matches', matches_frame))
					if draw_hough_lines and not(isinstance(boundary_error, DroneVisionError)):
						plot_frames.append(('hough_lines', hough_frame))
					if draw_detected_points:
						plot_frames.append(('points', delta_frame_l))
					self.__realTimePlot(plot_frames)
			#----------------------------------------#

			#---- DELETE FRAMES TO FREE MEMORY ----#
			del frame_un_l
			del delta_frame_l
			del matches_frame
			#----------------------------------------#

			##########################################

		self.ForceTermination()
		self.PrintFinished()
		self.RequestStop()
		self.SendFinishRequest()
		self.CloseMaster()

	def SendFinishRequest(self):
		'''
		 @brief Send finish request from master to slave
		'''
		if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
			if (self.GetSettings('REAL_TIME_PLOT', 'use_matplotlib') and self.GetSettings('REAL_TIME_PLOT', 'use_interactive_mode')) or not(self.GetSettings('REAL_TIME_PLOT', 'use_matplotlib')):
				getpass('Hit enter to terminate..')
		self.RequestDisconnect()

	def CheckRunMasterCalibration(self, calibrate_stereopsis=False, calibrate_blob_scale_detector=False):
		'''
		 @brief Check if a new calibration session should be started.

		 @param calibrate_stereopsis (Set calibrate_stereopsis to True for starting a new stereopsis calibration session (default=False))
		 @param calibrate_blob_scale_detector (Set calibrate_blob_scale_detector to True for starting a new blob scale detector calibration session (default=False))
		'''
		n_stereo_saved_calib_frame_sets = 0
		n_blob_saved_calib_frame_sets 	= 0
		if calibrate_stereopsis:
			calib_folder = self.GetSettings('CALIB', 'calib_img_folder_left_cam')
			n_stereo_saved_calib_frame_sets += self.RunMasterCalibration(calib_folder, True)
		if calibrate_blob_scale_detector:
			calib_folder = self.GetSettings('BLOB_SCALE', 'scale_calib_folder')
			n_blob_saved_calib_frame_sets += self.RunMasterCalibration(calib_folder, False)
		if (calibrate_stereopsis or calibrate_blob_scale_detector) and not(self.GetSettings('BASIC', 'reset_calibration')):
			self.__force_stereo_vision_calibration = self.GetYesNoFromUser('Force new stereopsis calibration with new calibration samples?')
			self.__force_blob_calibration = self.GetYesNoFromUser('Force new blob calibration with new calibration samples?')
			self.SendFlagToSlave(self.__force_stereo_vision_calibration)
			self.SendFlagToSlave(self.__force_blob_calibration)
		if self.GetSettings('REAL_TIME_PLOT', 'real_time_plot_on'):
			self.__realTimePlot(reset=True)

	def RunMasterCalibration(self, calib_folder, stereo_or_blob_calib, calib_filename='calib_frame'):
		'''
		 @brief Run master calibration.
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
		while not(self.CheckFinished()):
			self.SendFlagToSlave(True) # Send flag to slave commanding it to continue
			self.RequestFrameProcessingOnSlave() # Trigger slave to wait for new frame capturing.
			try:
				frame, sl_frame = self.GetRawFrames(get_normal_frame_only=get_normal_frame_only)
			except PtGreyError, err:
				warnings.simplefilter('always')
				warnings.warn(str(err), Warning)
				warnings.simplefilter('default')
				self.RestartCamera()
				self.RequestRestartPtGrey()
				continue
			self.WaitSlaveReady()

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

			answer = self.GetYesNoFromUser('Store images to calibration folder?')
			self.SendFlagToSlave(answer)
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
				print 'Master is saving calibration frames [{0}]..'.format(frame_n+1)
				if get_normal_frame_only:
					WriteImage(frame, calib_folder + calib_filename + '_' + str(frame_n))
				else:
					WriteImage(frame, normal_folder + calib_filename + '_' + str(frame_n))
					WriteImage(sl_frame, sl_folder + calib_filename + '_' + str(frame_n))
				frame_n += 1
			else:
				print 'Master did not save calibration frames..'
		self.SendFlagToSlave(False) # Send flag to slave commanding it to break
		if get_normal_frame_only: # Transfer all stereopsis images back and forth to slave/master
			if frame_n > 0:
				RemoveDir(self.GetSettings('CALIB', 'calib_img_folder_right_cam'))
				CheckDir(self.GetSettings('CALIB', 'calib_img_folder_right_cam'))
			for n_traded_frame in range(frame_n):
				traded_frame = GetImage(calib_folder + calib_filename + '_' + str(n_traded_frame) + '.tif')
				print 'Trading frame {0}/{1} with slave..'.format(n_traded_frame+1, frame_n)
				traded_frame, valid, error = self.RequestTradeFrame(self.GetSettings('CALIB', 'calib_img_folder_right_cam') + calib_filename + '_' + str(n_traded_frame) + '.tif', traded_frame)
				if not(valid) or error:
					raise Exception('Failed trading frames with slave')
				WriteImage(traded_frame, self.GetSettings('CALIB', 'calib_img_folder_right_cam') + calib_filename + '_' + str(n_traded_frame))
			self.SendFlagToSlave(True) # Send flag to slave commanding it to break
		return frame_n

	def ProcessCV(self, draw_heading=False, draw_hough_lines=False, draw_detected_points=False, draw_matches=False):
		'''
		 @brief Process computer vision steps

		 @param draw_heading (default=False)
		 @param draw_hough_lines (default=False) - draw_hough_lines overwrites draw_detected_points
		 @param draw_detected_points (default=False)
		 @param draw_matches (default=False)

		 @return points_error, boundary_error, heading_error, stereo_error, cv_results (Returns: points_error, heading_error, stereo_error (None, if no error and points_error as dominant error), cv_results = tuple containing elements of desired results.)
		'''
		points_error, frame_un_l, delta_frame_l, keypoints_l, descriptors_l, und_shape_r, keypoints_r, descriptors_r = self.GetProcessedFrames(draw_detected_points=draw_detected_points)
		if points_error != None:
			return points_error, None, None, None, None, None, None, None, None, None, None # Return failed frames.
		boundary_error, heading_error, heading_distance, heading_angle, frame_un_l, hough_frame = self.ProcessHeading(frame_un_l, delta_frame_l, keypoints_l, draw_heading=draw_heading, draw_hough_lines=draw_hough_lines)
		stereo_error, points3D, matches_frame = self.ProcessStereopsis(GetShape(frame_un_l), und_shape_r, keypoints_l, descriptors_l, keypoints_r, descriptors_r, draw_matches=draw_matches)
		return None, boundary_error, heading_error, stereo_error, heading_distance, heading_angle, points3D, frame_un_l, delta_frame_l, hough_frame, matches_frame

	def GetProcessedFrames(self, draw_detected_points=False):
		'''
		 @brief Get processed left and right frame simultaneously.
		 	Returns error if an error occurs (error=None if not).

		 @param draw_detected_points (default=False)

		 @return error, frame_un_l, delta_frame_l, keypoints_l, descriptors_l, frame_un_r_shape, keypoints_r, descriptors_r
		'''
		self.RequestFrameProcessingOnSlave() # Trig slave to capture new frames triggered by the master.
		try:
			original_frame_l, original_sl_frame_l = self.GetRawFrames() # Get new frames from slave, which triggers new frames to be captured on slave
			if self.GetSettings('DATABASE', 'store_frames_as_video') or self.GetSettings('DATABASE', 'store_frames_as_images'): # Store frames here to relieve memory. The frames are deleted as soon as possible.
				self.SetProcessFrame('original_left', original_frame_l)
				self.SetProcessFrame('original_sl_left', original_sl_frame_l)
				self.RecordData(record_frames=True, insert_to_database=False)
		except PtGreyError, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
			self.RestartCamera()
			self.RequestRestartPtGrey()
			return err, None, None, None, None, None, None, None
		t = RunThread(self.RequestPointlistThread)
		try:
			original_frame_l, original_sl_frame_l, frame_un_l, delta_frame_l, keypoints_l, descriptors_l = self.GetProcessedFrame(original_frame=original_frame_l, original_sl_frame=original_sl_frame_l, draw_detected_points=draw_detected_points) #Master is positioned to the left (left frame)
			del original_frame_l
			del original_sl_frame_l
			t.join() # Wait for slave point list request to finish
			if not(self.__req_success):
				if self.__req_error:
					self.RequestRestartPtGrey()
				raise DroneVisionError('could_not_get_point_list_from_slave')
		except DroneVisionError, err:
			self.__break_req = True # Break slave point list request if it's still running.
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')
			t.join() # Wait for slave point list request to terminate
			return err, None, None, None, None, None, None, None
		return None, frame_un_l, delta_frame_l, keypoints_l, descriptors_l, self.__frame_un_r_shape, self.__keypoints_r, self.__descriptors_r

	def RequestPointlistThread(self):
		'''
		 @brief Request point list from slave.
		 	Execute in thread.
		'''
		self.__break_req   	= False
		self.__req_success 	= False
		self.__req_error 	= False
		timeout = timeit.default_timer()
		while (not(self.__req_success) and not(self.__req_error) and (timeit.default_timer() - timeout) < self.GetSettings('TCP', 'frame_req_timeout')) and not(self.__break_req):
			self.__frame_un_r_shape, self.__keypoints_r, self.__descriptors_r, self.__req_success, self.__req_error = self.RequestPointList() #Slave is positioned to the right (right frame)

	def CheckFinished(self):
		'''
		 @Check if master is finished.

		 @return True/False - True = Finished, False = Not Finished
		'''
		stop = False
		if self.CheckDroneVisionFinished() or self.CheckTerminated():
			stop = True
		return stop #else

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
		self.ChangeSetting('DATABASE', 'output_folder', GetTimestampedFolder(timestamp, self.GetSettings('DATABASE', 'output_folder'), self.GetSettings('DATABASE', 'sub_output_folder')))

	def WaitSlaveReady(self):
		'''
		 @brief Wait for slave to be ready.
		'''
		slave_ready = False
		timeout = timeit.default_timer()
		while not(slave_ready) and ((timeit.default_timer() - timeout) < self.GetSettings('CALIB', 'calib_timeout') or self.GetSettings('CALIB', 'calib_timeout') < 0):
			slave_ready = self.RequestSlaveReady()
			if not(slave_ready):
				time.sleep(0.1)
		if not(slave_ready):
			raise Exception('Slave could not be ready within the timeout.')

	def CloseMaster(self):
		'''
		 @brief Close master safely
		'''
		Master.__del__(self)
		DataBase.__del__(self)

	def __del__(self):
		'''DESTRUCTOR'''
		self.CloseMaster()
