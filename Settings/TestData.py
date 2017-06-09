'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

import glob, warnings, os

'''
 @brief Class for getting test sets
 	Change data sets as preferred to use for testing.
'''
class TestData():
	def __init__(self):
		'''CONSTRUCTOR'''
		self.InitTestData()

	def InitTestData(self):
		'''
		 @brief Initialize test data
		'''
		self.__frame_sets = []
		self.ComputeFrameSets()

		###### INPUT VARIABLES ########

		###### GENREAL TEST INPUTS #######
		self.save_figs 									= False
		self.save_figs_only 							= False # Set true to not show figs during testing, save them only.
		self.savefig_folder 							= '../WindTurbineInspection_data/matplot_test_figs/'

		###### DRONEVISION TEST #######
		self.store_process_data 						= False
		self.store_frames_as_video 						= False
		self.store_frames_as_images 					= True
		self.draw_frames 								= True
		self.draw_heading 								= True
		self.print_progress								= True
		self.store_frames 								= False
		self.sub_output_folder 							= 'DroneMasterSlaveTests/'
		self.table_name 								= 'DroneMasterSlaveTests'
		self.source_type								= 'IMAGE'
		self.n_camera_frames 							= 10
		self.real_time_plot_on 							= True
		self.real_time_plot_matplotlib 					= True
		self.automatic_mode 							= False
		self.master_ip 									= 'localhost'

		##### STEREO VISION TEST #####
		self.show_delta_frames 							= False
		self.use_triangulation 							= False
		self.use_opencv_triangulation 					= True
		self.reset_calibration 							= True
		self.filtrate_3Dpoints 							= False
		self.test_FLANN_matching 						= False
		self.test_BRUTE_FORCE_matching 					= False

		######## DATABASE TEST ########
		self.username 									= 'root'
		self.password 									= 'odroid'
		self.database 									= 'WindTurbine_CV'
		self.host										= 'localhost'

		####### RECORD VIDEO TEST #####
		self.max_rec_frames								= 10
		self.vid_rec_fps 								= 4
		self.vid_rec_folder 							= '../samples_output/Test_RecordFrames/' 
		self.video_rec_output_fname 					= 'Test_RecordFrames'

		###### VIDEO TEST #############
		self.max_frames 								= 5
		self.input_video 								= '../samples/vid_output.avi'
		self.input_video_sl 							= '../samples/vid_output - copy.avi'

		####### PIN CONTROL TEST ###### 
		self.test_delay									= 5 #seconds
		###############################

		####### PTG CAMERA TEST ###### 
		self.slave_key 									= 'camera_slave' # Append this key word with the camera test to flag slave instance.
		self.manual_triggering 							= False
		self.camera_capture_timeout						= 10.0
		self.ptgrey_grab_infinite 						= True
		###############################

	def ComputeFrameSets(self):
		'''
		 @brief Compute test frame sets
		 	Add frame sets for testing by following the syntax.
		'''

		################################################## FINAL TEST SAMPLES #################################################################

		######### SPECIFIC SAMPLE SET ###########

		use_set 		= False
		actual_distance = -1.0 # mm (-1 means that the object is too close to be measured using stereopsis)
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/edge_detection/blade/blade/'
		left_frames_norm 	= ['left_camera/recordings/original_left_frames/original_left_3.tif', 'left_camera/recordings/original_left_frames/original_left_5.tif']
		left_frames_sl 		= ['left_camera/recordings/original_sl_left_frames/original_sl_left_3.tif', 'left_camera/recordings/original_sl_left_frames/original_sl_left_5.tif']
		right_frames_norm 	= ['right_camera/recordings/original_right_frames/original_right_3.tif', 'right_camera/recordings/original_right_frames/original_right_5.tif']
		right_frames_sl 	= ['right_camera/recordings/original_sl_right_frames/original_sl_right_3.tif','right_camera/recordings/original_sl_right_frames/original_sl_right_5.tif']
		
		self.CreateFrameSet(folder, \
			left_frames_norm, \
			left_frames_sl, \
			right_frames_norm, \
			right_frames_sl, \
			baseline, actual_distance, use_set)

		#################################

		######### BLADE TIP SET ###########

		use_set 		= True
		actual_distance = -1.0 # mm (-1 means that the object is too close to be measured using stereopsis)
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/edge_detection/blade/blade_tip/2016_02_11__17_20_34/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################

		######### MORE OF BLADE SET ###########

		use_set 		= False
		actual_distance = -1.0 # mm (-1 means that the object is too close to be measured using stereopsis)
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/edge_detection/blade/more_of_blade/2016_02_11__18_40_33/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################

		######### LESS OF BLADE SET ###########

		use_set 		= False
		actual_distance = -1.0 # mm (-1 means that the object is too close to be measured using stereopsis)
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/edge_detection/blade/less_of_blade/2016_02_11__18_33_13/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################

		######### RANDOM BLADE SET ###########

		use_set 		= False
		actual_distance = -1.0 # mm (-1 means that the object is too close to be measured using stereopsis)
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/edge_detection/blade/random_blade/2016_02_11__17_54_12/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################

		######### ABSORBING BLADE COLOR ###########

		use_set 		= False
		actual_distance = -1.0 # mm (-1 means that the object is too close to be measured using stereopsis)
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/edge_detection/blade/absorbing_blade_color/2016_02_11__17_34_12/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################

		######### STEREOPSIS SET ###########

		use_set 		= False
		actual_distance = [1245.0, 1640.0] # mm
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/stereopsis/dist_124cm_164cm/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################

		######### STEREOPSIS SET ###########

		use_set 		= False
		actual_distance = 0.0 # mm
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/stereopsis/dist_full_test/2016_02_11__16_42_21/'
		#folder 			= '../WindTurbineInspection_data/final_test_samples/stereopsis/dist_full_test/2016_02_11__17_17_15/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################

		######### BOX CARTON SET ###########

		use_set 		= False
		actual_distance = 1050.0 # mm
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/edge_detection/objects/box_carton_dist_105cm/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################

		######### SQUARE POLE SET ###########

		use_set 		= False
		actual_distance = -1.0 # mm (-1 means that the object is too close to be measured using stereopsis)
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/edge_detection/objects/square_pole/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################


		######### BLADE SET ###########

		use_set 		= False
		actual_distance = -1.0 # mm (-1 means that the object is too close to be measured using stereopsis)
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/edge_detection/blade/blade/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################

		######### BLADE (WINDOW) SET ###########

		use_set 		= False
		actual_distance = -1.0 # mm (-1 means that the object is too close to be measured using stereopsis)
		baseline 		= 50.0 	# mm
		folder 			= '../WindTurbineInspection_data/final_test_samples/edge_detection/blade/blade_window/'

		self.CreateFrameSet(folder, \
			'left_camera/recordings/original_left_frames/', \
			'left_camera/recordings/original_sl_left_frames/', \
			'right_camera/recordings/original_right_frames/', \
			'right_camera/recordings/original_sl_right_frames/', \
			baseline, actual_distance, use_set)

		#################################

	def CreateFrameSet(self, folder, left_normal_folder, left_sl_folder, right_normal_folder, right_sl_folder, baselines=50.0, actual_distances=-1.0, use_set=True, file_type='*'):
		'''
		 @brief Create test frame set

		 @param folder (Top folder for the test set)
		 @param left_normal_folder (folder for left normal frames)
		 @param left_sl_folder (folder for left sl frames)
		 @param right_normal_folder (folder for right normal frames)
		 @param right_sl_folder (folder for right sl frames)
		 	- Note all of the parameters above (except from 'folder') may also be given as consistent lists with specific test filenames.
		 @param baselines (List of baselines for given test set in mm. May also be a fixed number. (default=50.0))
		 @param actual_distances (List of actual distances for each frame set of the frame sets. May also be a fixed number. (default=-1 - unknown actual distance))
		 @param use_set (True/False for using this test for testing (default=True))
		 @param file_type (Type of files in given folder. Note that all files will be included from the folder, as specified by the file type. (default='*' - means all file types))
		'''
		left_frames			= []
		right_frames 		= []

		try: # Just if user gives some corrupt test sets
			if isinstance(left_normal_folder, list) or isinstance(left_sl_folder, list) or isinstance(right_normal_folder, list) or isinstance(right_sl_folder, list): # Specific test sets are given as list
				if not(isinstance(left_normal_folder, list) and isinstance(left_sl_folder, list) and isinstance(right_normal_folder, list) and isinstance(right_sl_folder, list)):
					raise Exception('All sets must be given as lists, if either one of them are given as a list. Another option is to give them as folders.')
				n_sets = len(left_normal_folder)
				if not(len(left_sl_folder) == n_sets and len(right_normal_folder) == n_sets and len(right_sl_folder) == n_sets):
					raise Exception('Number of test files are not consistent.')
				for i in range(n_sets):
					if not(os.path.isfile(folder + left_normal_folder[i]) and os.path.isfile(folder + left_sl_folder[i]) and os.path.isfile(folder + right_normal_folder[i]) and os.path.isfile(folder + right_sl_folder[i])):
						raise Exception('One of the files given does not exist, check: {0}, {1}, {2}, {3}'.format(left_normal_folder[i], left_sl_folder[i], right_normal_folder[i], right_sl_folder[i]))
					left_frames.append((left_normal_folder[i], left_sl_folder[i]))
					right_frames.append((right_normal_folder[i], right_sl_folder[i]))
				test_folder = folder
			else:
				left_frames_norm 	= glob.glob(folder+left_normal_folder+'*.'+file_type)
				left_frames_sl 		= glob.glob(folder+left_sl_folder+'*.'+file_type)
				right_frames_norm 	= glob.glob(folder+right_normal_folder+'*.'+file_type)
				right_frames_sl 	= glob.glob(folder+right_sl_folder+'*.'+file_type)
				n_sets 				= len(left_frames_norm)
				if not(len(left_frames_sl) == n_sets and len(right_frames_norm) == n_sets and len(right_frames_sl) == n_sets):
					raise Exception('Number of test files are not consistent in the test folders.')
				for i in range(n_sets):
					left_frames.append((left_frames_norm[i], left_frames_sl[i]))
					right_frames.append((right_frames_norm[i], right_frames_sl[i]))
				test_folder = ''

			if not(isinstance(actual_distances, list)):
				actual_distances = [actual_distances]*len(left_frames)
			if not(isinstance(baselines, list)):
				baselines = [baselines]*len(left_frames)
		except Exception, err:
			error_msg = 'Failed creating test set from folder: {0} -> {1}'.format(folder, str(err))
			warnings.simplefilter('always')
			warnings.warn(error_msg, Warning)
			warnings.simplefilter('default')
			return

		self.AppendFrameSet(test_folder, left_frames, right_frames, actual_distances, baselines, use_set)

	def AppendFrameSet(self, folder, left_frames, right_frames, actual_distances, baselines, use_set=True):
		'''
		 @brief Append test frame set to list of test frame sets
		 	Left. right frames and corresponding distances must be consistent.

		 @param folder (folder to listed frames)
		 @param left_frames (List of touples as [(left_fn_frame, left_fn_sl_frame)], fn_frame = filename without structured light, and fn_sl_frame is with structured light)
		 @param right_frames (List of touples as [(right_fn_frame, right_fn_sl_frame)], fn_frame = filename without structured light, and fn_sl_frame is with structured light)
		 @param baselines (List of baselines)
		 @param actual_distances (List of actual distances)
		 @param use_set (flag for using set when testing)
		'''
		self.__frame_sets.append((folder, left_frames, right_frames, actual_distances, baselines, use_set))

	def GetFrameSets(self):
		'''
		 @brief Get test frame sets

		 @return list of frame sets as [(folder, left_frames, right_frames, actual_distances, baselines,  use_set)]
		'''
		return self.__frame_sets
