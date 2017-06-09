'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

'''
 @brief SettingsConfigured class
  Change default settings as preferred, but do not change the setting types or structure.

 @param load_initial_settings (default=True)
'''
class SettingsConfigured():
	def __init__(self, load_initial_settings=True):
		'''CONSTRUCTOR'''
		self.SetStaticSettings()
		if load_initial_settings:
			self.SetInitialSettings()

	def SetStaticSettings(self):
		'''
		 @brief Set static settings
		'''
		self._static_settings								= {}
		self._static_settings['reset_settings'] 			= False # Reset settings to default settings (given by this script)
		self._static_settings['settings_params_folder']		= '../WindTurbineInspection_data/settings_configuration/'
		self._static_settings['settings_params_fname'] 		= 'settings_configuration'

	def GetStaticSettingsInfo(self):
		'''
		 @brief Get static settings info

		 @return static_settings_info
		'''
		static_settings_info 								= {}
		static_settings_info['reset_settings'] 				= "Reset settings to default setting values as specified by 'SettingsConfigured.py', options: True/False"
		static_settings_info['settings_params_folder']		= "Folder path for the saved settings configuration file, options: (string) - folder path as 'settings_folder/'"
		static_settings_info['settings_params_fname'] 		= "Settings configuration filename without file ending, options: (string) - filename as 'settings_config_saved'"
		return static_settings_info

	def SetInitialSettings(self):
		'''
		 @brief Set initial fixed settings
		 	settings is comprised as follows:
		 		settings[header][setting] = value
		 	header 	-> settings area
			setting -> setting key in area
		'''
		self._settings = {}
		##################################################

		#---- BASIC SETTINGS -----#
		self._settings['BASIC'] 									= {}
		self._settings['BASIC']['source_type'] 						= 'IMAGE' 	#Options: 'IMAGE' 'VIDEO' or 'CAMERA' 
		self._settings['BASIC']['reset_calibration'] 				= False
		self._settings['BASIC']['master_timeout'] 					= 20.0 # Seconds for slave to raise master timeout error if master doesn't send request before timeut. Is only on if no user inputs blocks automatic mode.
		#---- REAL TIME PLOT -----#
		self._settings['REAL_TIME_PLOT'] 							= {}
		self._settings['REAL_TIME_PLOT']['real_time_plot_on'] 		= True
		self._settings['REAL_TIME_PLOT']['use_matplotlib'] 			= False # False will result in using pyQtGraph which is much faster.
		self._settings['REAL_TIME_PLOT']['use_interactive_mode'] 	= True # True will result in interactive mode when using matplotlib (non-blocking figures)
		self._settings['REAL_TIME_PLOT']['save_fig_folder'] 		= 'matplotlib_fig_results/' # Folder for saving real-time figure plots if display is unavailable. Folder will be set under designated (timestamped) output folder.
		#---- USER INPUT SETTINGS ----#
		self._settings['USER_INPUT'] 								= {}
		self._settings['USER_INPUT']['automatic_mode'] 				= True 	# Set True to run program without any breaking point.
		self._settings['USER_INPUT']['no_termination'] 				= True 	# Set True to run program with no termination possibility.
		self._settings['USER_INPUT']['key_terminate']				= 'q'	# Press/hold this key to terminate the program.
		#---- COMPUTER VISION SETTINGS ----#
		self._settings['CV'] 										= {}
		self._settings['CV']['rho_step_distance'] 					= None  		# If None = then each step distance (how far the drone should fly during each step) is set to a quarter the diagonal frame size
		self._settings['CV']['rho_min_diag_perc'] 					= 1/4.0 		# Percent of the diagonal length from center to image edge. Used to calculate rho_min between the blade edge and image edge.
		self._settings['CV']['default_downsampling_divisor'] 		= 4 			# Default downsampling divisor.
		self._settings['CV']['desired_frame_shape'] 				= (512,612) 	# Desired frame shape to work with, given as a tuple of (height, width). Set to (-1,-1) to not use a desired frame shape, and instead stay fixed to the default downsampling divisor.
		self._settings['CV']['detector_type'] 						= 0 			# Detector type to use for detecting feature points (blobs) - options: 0, 1, 2, 3 for simple blob detector, ORB, SIFT or SURF (in that order)
		self._settings['CV']['crop_frames'] 						= True 			# Crop frames according to the difference in fan angle between the laser and camera.
		#---- DATABASE SETTINGS ----#
		self._settings['DATABASE'] 									= {}
		self._settings['DATABASE']['username'] 						= 'root' 					# Set to None to make user type in username at bootup
		self._settings['DATABASE']['password'] 						= 'odroid' 					# Set to None to make user type in password at bootup
		self._settings['DATABASE']['database'] 						= 'WindTurbine_CV' 			# Set to None to make user type in the database at bootup
		self._settings['DATABASE']['table_name']					= '' 						# Set to None to make user type in table name at bootup. The table will be appended with a timestamp.
		self._settings['DATABASE']['output_folder']					= 'DataSamples/samples_output/' 		# Set to None to make user type in output folder at bootup
		self._settings['DATABASE']['sub_output_folder']				= '' 					# Give the output data folder additional subfolder (f.ex 'sub_folder/'). Set '' for no subfolder. Set to None to make user type in sub folder at bootup
		self._settings['DATABASE']['print_progress'] 				= True
		self._settings['DATABASE']['print_3D_points'] 				= False 		# Print all 3D point values (does not save to database)
		self._settings['DATABASE']['store_process_data'] 			= False 	# Store process data to database
		self._settings['DATABASE']['draw_heading'] 					= True		# Draw headings and information on incoming frames
		self._settings['DATABASE']['draw_matches'] 					= False 		# Draw stereopsis matches
		self._settings['DATABASE']['draw_hough_lines']  			= True 		# Draw hough lines 
		self._settings['DATABASE']['draw_detected_points'] 			= True 		# Draw detected points
		self._settings['DATABASE']['store_drawn_frames'] 			= False 	# Store drawn frames, next to the original frames.
		self._settings['DATABASE']['store_frames_as_video'] 		= False 		# Store frames as videos (Separate video for each frame set)
		self._settings['DATABASE']['store_frames_video_fps']		= 1.0		# FPS for stored videos
		self._settings['DATABASE']['store_frames_as_images'] 		= False 	# Store fames as images (Separate folder for each frame set)
		#---- CAMERA CALIBRATION SETTINGS ----#
		self._settings['CALIB'] 									= {}
		self._settings['CALIB']['calib_img_folder_left_cam'] 		= 'DataSamples/calibration_samples/camera_calib_samples/left_camera/'
		self._settings['CALIB']['calib_img_folder_right_cam'] 		= 'DataSamples/calibration_samples/camera_calib_samples/right_camera/'
		self._settings['CALIB']['calib_img_type']					= '*'
		self._settings['CALIB']['calib_save_folder']				= 'DataSamples/calibration_saves/camera_calibration/'
		self._settings['CALIB']['calib_save_fname_left_cam'] 		= 'left_calib_param'
		self._settings['CALIB']['calib_save_fname_right_cam'] 		= 'right_calib_param'
		self._settings['CALIB']['calib_save_fname_stereo'] 			= 'stereo_calib_param'
		self._settings['CALIB']['calib_show_imgs']					= False
		self._settings['CALIB']['calib_print_process']				= True
		self._settings['CALIB']['calib_chess_rows']					= 6
		self._settings['CALIB']['calib_chess_columns']				= 9
		self._settings['CALIB']['save_calib_param_to_json'] 		= True
		self._settings['CALIB']['calib_timeout'] 					= -1 # sec - Set < 0 for inf
		self._settings['CALIB']['focal_length'] 					= 8.5 	#mm
		self._settings['CALIB']['baseline']		 					= 50.0 	# Baseline (mm)
		self._settings['CALIB']['sensor_size'] 						= (6.6, 8.8) #(height, width) in mm of the sensor size.
		#---- FEATURE STEREOPSIS SCALE SETTINGS -----#
		self._settings['F_STEREO'] 									= {}
		self._settings['F_STEREO']['use_triangulation'] 			= False 	# Use triangulation to compute 3D point coordinates. False will use easy f*T/d computation.
		self._settings['F_STEREO']['use_cv2_triangulation']			= True 		# Use opencv implemented triangulation algorithm. False will result in using the Harley & Zisserman method.
		self._settings['F_STEREO']['use_block_matching'] 			= True 		# Use block matching method to match feature points. False will use FLANN based or brute forced based matching.
		self._settings['F_STEREO']['block_matching_parameter'] 		= 2.5 		# Block matching parameter. Increase/decrease the scaling parameter for finding matches.
		self._settings['F_STEREO']['use_brute_force'] 				= False 	# True/False for using brute force matching instead of FLANN based matching (only active if use_block_matching=False)
		#---- BLOB SCALE SETTINGS -----#
		self._settings['BLOB_SCALE'] 								= {}
		self._settings['BLOB_SCALE']['scale_calib_folder']			= 'DataSamples/calibration_samples/blob_scale_calib_samples/'
		self._settings['BLOB_SCALE']['scale_img_type'] 				= '*'
		self._settings['BLOB_SCALE']['scale_calib_save_folder']		= 'DataSamples/calibration_saves/scale_calibration/'
		self._settings['BLOB_SCALE']['scale_calib_save_fname'] 		= 'scale_param' 
		self._settings['BLOB_SCALE']['scale_filtrate'] 				= True
		#---- MASTER / SLAVE TCP -----#
		self._settings['TCP']										= {}
		#self._settings['TCP']['master_ip']							= '192.168.137.54'
		self._settings['TCP']['master_ip']							= 'localhost'
		self._settings['TCP']['port'] 								= 1991
		self._settings['TCP']['master_buffer_size']					= 3072 	# Max = 8192
		self._settings['TCP']['slave_buffer_size']					= 256 	# Min = 128
		self._settings['TCP']['tcp_timeout']						= 10.0 	# Tcp timeout in sec
		self._settings['TCP']['frame_req_timeout']					= 10.0
		self._settings['TCP']['print_payload_info'] 				= False # Set True to print payload information during runtime. Information gives how large the payloads are, to adjust the buffer sizes.
		#---- CAMERA SETTINGS ----#
		self._settings['CAMERA'] 									= {}
		self._settings['CAMERA']['ptgrey_library'] 					= 'Jordens' # Tag for selecting which library to use. Options - FLIR library: 'FLIR', Jordens library: 'Jordens'. Jordens library is set as default. (Woops.. The FLIR library isn't finished implemented (4/6/17))
		self._settings['CAMERA']['auto_camera_config'] 				= False # Set true to enable the PtGrey camera to auto calibrate camera settings during process. See CameraLink module for setting initial settings.
		self._settings['CAMERA']['manual_triggering'] 				= False # Set to True for manual camera triggering. Pauses the program and waits for user to hit enter to continue. 
		self._settings['CAMERA']['n_frames'] 						= -1 # Set <= 0 for indefinite
		self._settings['CAMERA']['ptg_triggerPin'] 					= 2  # Set -1 to disable - trigger pin on the Ptg. Usually GPIO3 (Purple wire)
		self._settings['CAMERA']['ptg_triggerOutPin']				= 3  # Set -1 to disable - trigger out pin on the Ptg. Usually GPIO3 (Green wire)
		self._settings['CAMERA']['camera_triggerPin'] 				= 22 # trigger pin on the odroid to the camera
		self._settings['CAMERA']['ptg_recv_frame_timeout']			= 1.5
		self._settings['CAMERA']['ptg_use_color_frames'] 			= True # Set Ptg to capture color frames
		self._settings['CAMERA']['fan_angle'] 						= 60.0 # fan angle of the camera in cartesian degrees (actuall camera fan angle is 54.72, but increasing it to make sure all points covers the whole image)
		#----- LASER SETTINGS ----#
		self._settings['LASER'] 									= {}
		self._settings['LASER']['laser_triggerPin'] 				= 26 # Trigger pin on the odroid to the laser
		self._settings['LASER']['fan_angle'] 						= 28.2 # fan angle of the camera in cartesian degrees
		#---- VIDEO SETTINGS -----#
		self._settings['VIDEO'] 									= {}
		self._settings['VIDEO']['input_folder']						= 'DataSamples/live_test_sample/'
		self._settings['VIDEO']['left_video']						= 'left_camera/recordings/original_left.avi' 			# Master is to the left
		self._settings['VIDEO']['left_sl_video']					= 'left_camera/recordings/original_sl_left.avi' 
		self._settings['VIDEO']['right_video']						= 'right_camera/recordings/original_right.avi'			# Slave is to the right
		self._settings['VIDEO']['right_sl_video']					= 'right_camera/recordings/original_sl_right.avi'
		#---- IMAGE SETTINGS -----#
		self._settings['IMAGE'] 									= {}
		self._settings['IMAGE']['input_folder']						= 'DataSamples/live_test_sample/'
		self._settings['IMAGE']['left_images']						= 'left_camera/recordings/original_left_frames/' 			# Master is to the left (May also be list of filenames - consistent with following input images)
		self._settings['IMAGE']['left_sl_images']					= 'left_camera/recordings/original_sl_left_frames/' 
		self._settings['IMAGE']['right_images']						= 'right_camera/recordings/original_right_frames/'	# Slave is to the right
		self._settings['IMAGE']['right_sl_images']					= 'right_camera/recordings/original_sl_right_frames/'

		##################################################

	def GetSettingsInfo(self):
		'''
		 @brief Get settings info as a dictionary.

		 @return settings_info
		'''
		settings_info = {}
		##################################################

		#---- BASIC SETTINGS -----#
		settings_info['BASIC'] 										= {}
		settings_info['BASIC']['source_type'] 						= "Choose source type, options: 'IMAGE' 'VIDEO' or 'CAMERA'"
		settings_info['BASIC']['reset_calibration'] 				= "Reset calibration, options: True/False"
		settings_info['BASIC']['master_timeout'] 					= "Seconds for slave to raise timeout error, raised if master havent sent any requests during this timeout. It is only on in automatic mode. Options: (float) seconds"
		#---- REAL TIME PLOT -----#
		settings_info['REAL_TIME_PLOT'] 							= {}
		settings_info['REAL_TIME_PLOT']['real_time_plot_on'] 		= "Turn real-time plot On or Off, options: True/False"
		settings_info['REAL_TIME_PLOT']['use_matplotlib'] 			= "Use matplotlib as real-time plot, Options: True/False. False will result in using pyQtGraph which is much faster."
		settings_info['REAL_TIME_PLOT']['use_interactive_mode'] 	= "Options: True/False. True will result in interactive mode when using matplotlib (non-blocking figures)"
		settings_info['REAL_TIME_PLOT']['save_fig_folder'] 			= "Folder for saving real-time figure plots if display is unavailable. Folder will be set under designated (timestamped) output folder, options: (string) - folder path as 'this_folder/'"
		#---- USER INPUT SETTINGS ----#
		settings_info['USER_INPUT'] 								= {}
		settings_info['USER_INPUT']['automatic_mode'] 				= "Options True/False. Set True to run program without any breaking point."
		settings_info['USER_INPUT']['no_termination']				= "Options True/False. Set True to run program with no possibility of termination."
		settings_info['USER_INPUT']['key_terminate']				= "Options: (char). Press/hold this key (char) to terminate the program."
		#---- COMPUTER VISION SETTINGS ----#
		settings_info['CV'] 										= {}
		settings_info['CV']['rho_step_distance'] 					= "Options: None/(float). If None = then each step distance (how far the drone should fly during each step) is set to a quarter the diagonal frame size"
		settings_info['CV']['rho_min_diag_perc'] 					= "Options: (float), between 0 -> 1. Percent of the diagonal length from center to image edge. Used to calculate rho_min between the blade edge and image edge."
		settings_info['CV']['default_downsampling_divisor'] 		= "Options: (int) - even number >= 1 giving the default divisor for downsampling incoming frames."
		settings_info['CV']['desired_frame_shape'] 					= "Desired frame shape to work with. Given as a tuple of (height, width). Set to (-1,-1) to not use a desired frame shape, and instead stay fixed to the default downsampling divisor."
		settings_info['CV']['detector_type']						= "Detector type to use for detecting feature points (blobs). Options (int): 0, 1, 2, 3 for simple blob detector, ORB, SIFT or SURF (in that order). Simple blob detector (option 0) is recommended."
		settings_info['CV']['crop_frames'] 							= "Options True/False. Crop frames according to the difference in fan angle between the laser and camera."
		#---- DATABASE SETTINGS ----#
		settings_info['DATABASE'] 									= {}
		settings_info['DATABASE']['username'] 						= "Database username (mysql), options: None/(string) - Set to None to make user type in username at startup"
		settings_info['DATABASE']['password'] 						= "Database password (mysql), options: None/(string) - Set to None to make user type in username at startup"
		settings_info['DATABASE']['database'] 						= "Database name (mysql), options: None/(string) - Set to None to make user type in username at startup"
		settings_info['DATABASE']['table_name']						= "Database table name (mysql), options: None/(string) - Set to None to make user type in username at startup. The table name will be appended with current timestamp."
		settings_info['DATABASE']['output_folder']					= "Main output folder for storing data (f.ex 'main_folder/'), options: None/(string) - Set to None to make user type in username at startup."
		settings_info['DATABASE']['sub_output_folder']				= "Give the main output folder an additional subfolder (f.ex 'sub_folder/'), options: None/(string). Set '' for no subfolder. Set to None to make user type in sub folder at startup"
		settings_info['DATABASE']['print_progress'] 				= "Print progress data to screen. Progress data is automatically written to a log.txt file, options: True/False"
		settings_info['DATABASE']['print_3D_points'] 				= "Print all 3D point values (does not save to mysql database), options: True/False"
		settings_info['DATABASE']['store_process_data'] 			= "Store process data to database (mysql), options: True/False"
		settings_info['DATABASE']['draw_heading'] 					= "Draw heading and detected blade edges, options: True/False"
		settings_info['DATABASE']['draw_matches'] 					= "Draw feature point matches from the stereopsis system, options: True/False"
		settings_info['DATABASE']['draw_hough_lines']  				= "Draw hough lines grid and bounded lines, options: True/False"
		settings_info['DATABASE']['draw_detected_points'] 			= "Draw detected feature points, options: True/False"
		settings_info['DATABASE']['store_drawn_frames'] 			= "Store drawn frames, next to the original frames, options: True/False. Original frames (with and without structured light) are stored if either store_frames_as_video or store_frames_as_images is set to True."
		settings_info['DATABASE']['store_frames_as_video'] 			= "Store frames as videos (Separate video for each frame set), options: True/False"
		settings_info['DATABASE']['store_frames_video_fps']			= "FPS for stored videos, options: (float)"
		settings_info['DATABASE']['store_frames_as_images'] 		= "Store fames as images (Separate folder for each frame set), options: True/False"
		#---- CAMERA CALIBRATION SETTINGS ----#
		settings_info['CALIB'] 										= {}
		settings_info['CALIB']['calib_img_folder_left_cam'] 		= "Folder path for camera calibration frames belonging to the left camera, options (string) - folder path as 'left_camera_calib_folder/'"
		settings_info['CALIB']['calib_img_folder_right_cam'] 		= "Folder path for camera calibration frames belonging to the right camera, options (string) - folder path as 'right_camera_calib_folder/'"
		settings_info['CALIB']['calib_img_type']					= "Calibration image types, options: (string) - f.ex '.tif', or '*' for any file type."
		settings_info['CALIB']['calib_save_folder']					= "Folder path for storing calibration parameters, options (string) - folder path as 'camera_calib_folder/'"
		settings_info['CALIB']['calib_save_fname_left_cam'] 		= "Filename for saving left camera calibration parameters, options (string) - filename without file ending as 'left_calib_param'"
		settings_info['CALIB']['calib_save_fname_right_cam'] 		= "Filename for saving right camera calibration parameters, options (string) - filename without file ending as 'right_calib_param'"
		settings_info['CALIB']['calib_save_fname_stereo'] 			= "Filename for saving stereo camera calibration parameters, options (string) - filename without file ending as 'stereo_calib_param'"
		settings_info['CALIB']['calib_show_imgs']					= "Show calibration images during calibration, options: True/False"
		settings_info['CALIB']['calib_print_process']				= "Print calibration progress during calibration, options: True/False"
		settings_info['CALIB']['calib_chess_rows']					= "Set number of rows on the calibration chess chart, options (int)"
		settings_info['CALIB']['calib_chess_columns']				= "Set number of columns on the calibration chess chart, options (int)"
		settings_info['CALIB']['save_calib_param_to_json'] 			= "Save calibration parameters as a readable json next to the pickle file, only used for readability. Options: True/False"
		settings_info['CALIB']['calib_timeout'] 					= "Master waits this many seconds for slave to finish calibrating, options: (float) - Set < 0 for infinite waiting time."
		settings_info['CALIB']['focal_length'] 						= "Focal length in mm of the cameras in use, options (float)."
		settings_info['CALIB']['baseline']		 					= "Baseline in mm between the cameras, options (float)"
		settings_info['CALIB']['sensor_size'] 						= "Sensor size in mm given as a tuple (height, width), options: (float,float)"
		#---- FEATURE STEREOPSIS SCALE SETTINGS -----#
		settings_info['F_STEREO'] 									= {}
		settings_info['F_STEREO']['use_triangulation'] 				= "Use triangulation to compute 3D point coordinates. False will use easy f*T/d stereo computation. Options: True/False"
		settings_info['F_STEREO']['use_cv2_triangulation']			= "Use opencv implemented triangulation algorithm. False will result in using the Harley & Zisserman method. Options: True/False"
		settings_info['F_STEREO']['use_block_matching'] 			= "Use block matching method to match feature points. False will use FLANN based or brute forced based matching. Options: True/False"
		settings_info['F_STEREO']['block_matching_parameter'] 		= "Block matching parameter. Increase/decrease the scaling parameter for finding matches. Options: (float)"
		settings_info['F_STEREO']['use_brute_force'] 				= "Use brute force matching instead of FLANN based matching (only active if use_block_matching is False), options: True/False"
		#---- BLOB SCALE SETTINGS -----#
		settings_info['BLOB_SCALE'] 								= {}
		settings_info['BLOB_SCALE']['scale_calib_folder']			= "Folder path for the standard distance between blobs calibration, options: (string) - folder path as 'blob_scale_calib/'"
		settings_info['BLOB_SCALE']['scale_img_type'] 				= "Blob calibration image types, options: (string) - f.ex '.tif', or '*' for any file type."
		settings_info['BLOB_SCALE']['scale_calib_save_folder']		= "Folder path for storing blob calibration parameters, options (string) - folder path as 'blob_scale_calib_param/'"
		settings_info['BLOB_SCALE']['scale_calib_save_fname'] 		= "Filename for saving blob calibration parameters, options (string) - filename without file ending as 'blob_scale_calib_param'"
		settings_info['BLOB_SCALE']['scale_filtrate'] 				= "Filtrate detected distances between blobs by removing distances outside of the standard deviation, options: True/False"
		#---- MASTER / SLAVE TCP -----#
		settings_info['TCP']										= {}
		settings_info['TCP']['master_ip']							= "IP address to the master device, options: <ip_address> - f.ex '192.168.137.54', or 'localhost' to run a simulation on a single device."
		settings_info['TCP']['port'] 								= "TCP port, options: (int)"
		settings_info['TCP']['master_buffer_size']					= "TCP buffer size on the master device, options: (int) - <min,max> = <128,8192>"
		settings_info['TCP']['slave_buffer_size']					= "TCP buffer size on the slave device, options: (int) - <min,max> = <128,8192>"
		settings_info['TCP']['tcp_timeout']							= "TCP send/receive timeout in seconds, options: (float)"
		settings_info['TCP']['frame_req_timeout']					= "Timeout in seconds for the master to wait for slave to process and send a frame set (or keypoints and descriptors), options: (float)"
		settings_info['TCP']['print_payload_info'] 					= "Options: True/False. Set True to print payload information during runtime. Information gives how large the payloads are, to adjust the buffer sizes. Should only be True during testing."
		#---- CAMERA SETTINGS ----#
		settings_info['CAMERA'] 									= {}
		settings_info['CAMERA']['ptgrey_library'] 					= "Tag for selecting which library to use. Options - FLIR library: 'FLIR', Jordens library: 'Jordens'. Jordens library is set as default. (Woops.. The FLIR library isn't finished implemented (4/6/17))"
		settings_info['CAMERA']['auto_camera_config'] 				= "Set true to enable the PtGrey camera to auto calibrate camera settings during process. See CameraLink module for setting initial camera settings. Options: True/False"
		settings_info['CAMERA']['manual_triggering'] 				= "Set to True for manual camera triggering. Pauses the program and waits for user to hit enter, triggering a frame capture. Options: True/False"
		settings_info['CAMERA']['n_frames'] 						= "Number of frames to process before the program automatically finishes, options: (int) - set < 0 to never stop."
		settings_info['CAMERA']['ptg_triggerPin'] 					= "Trigger pin on the PtGrey. Usually GPIO3 (Purple wire), options: (int) - Set -1 to disable the trigger."
		settings_info['CAMERA']['ptg_triggerOutPin']				= "Trigger out pin on the PtGrey. Usually GPIO3 (Green wire), options: (int) - Set -1 to disable the out trigger."
		settings_info['CAMERA']['camera_triggerPin'] 				= "GPIO pin on the odroid for triggering the PtGrey, options (int)."
		settings_info['CAMERA']['ptg_recv_frame_timeout']			= "Timeout to receive a captured frames from the PtGrey, options (float)"
		settings_info['CAMERA']['ptg_use_color_frames'] 			= "Set Ptg to capture color frames, options: True/False"
		settings_info['CAMERA']['fan_angle']						= "Fan angle of the camera in cartesian degrees, options (float)"
		#----- LASER SETTINGS ----#
		settings_info['LASER'] 										= {}
		settings_info['LASER']['laser_triggerPin'] 					= "GPIO pin on the odroid to turn the laser On/Off, options: (int)"
		settings_info['LASER']['fan_angle']							= "Fan angle of the laser in cartesian degrees, options (float)"
		#---- VIDEO SETTINGS -----#
		settings_info['VIDEO'] 										= {}
		settings_info['VIDEO']['input_folder']						= "Top folder for the given video sample. A video sample can be obtained be turning on video storing during a real process. Following video settings must be consistent. Options: (string) - folder path as 'video_sample/'"
		settings_info['VIDEO']['left_video']						= "Video of original samples from the left camera without structured light, options: (string) - f.ex 'original_left_avi'"
		settings_info['VIDEO']['left_sl_video']						= "Video of original samples from the left camera with structured light, options: (string) - f.ex 'original_sl_left_avi'"
		settings_info['VIDEO']['right_video']						= "Video of original samples from the right camera without structured light, options: (string) - f.ex 'original_right_avi'"
		settings_info['VIDEO']['right_sl_video']					= "Video of original samples from the right camera with structured light, options: (string) - f.ex 'original_sl_right_avi'"
		#---- IMAGE SETTINGS -----#
		settings_info['IMAGE'] 										= {}
		settings_info['IMAGE']['input_folder']						= "Top folder for the given image samples. Image samples can be obtained be turning on image storing during a real process. Following image settings must be consistent. Options: (string) - folder path as 'image_samples/'"
		settings_info['IMAGE']['left_images']						= "Image sets of original samples from the left camera without structured light. May be a single image file, a directory with files, or a list of image files, options: (string)/(string_dir/)/[(string_0),..,(string_n)] - f.ex 'original_left.tif', 'original_left_frames/' or ['original_left_0.tif',..,'original_left_n.tif']"
		settings_info['IMAGE']['left_sl_images']					= "Image sets of original samples from the left camera with structured light. May be a single image file, a directory with files, or a list of image files, options: (string)/(string_dir/)/[(string_0),..,(string_n)] - f.ex 'original_sl_left.tif', 'original_sl_left_frames/' or ['original_sl_left_0.tif',..,'original_sl_left_n.tif']"
		settings_info['IMAGE']['right_images']						= "Image sets of original samples from the right camera without structured light. May be a single image file, a directory with files, or a list of image files, options: (string)/(string_dir/)/[(string_0),..,(string_n)] - f.ex 'original_right.tif', 'original_right_frames/' or ['original_right_0.tif',..,'original_right_n.tif']"
		settings_info['IMAGE']['right_sl_images']					= "Image sets of original samples from the right camera with structured light. May be a single image file, a directory with files, or a list of image files, options: (string)/(string_dir/)/[(string_0),..,(string_n)] - f.ex 'original_sl_right.tif', 'original_sl_right_frames/' or ['original_sl_right_0.tif',..,'original_sl_right_n.tif']"

		return settings_info
		##################################################
		