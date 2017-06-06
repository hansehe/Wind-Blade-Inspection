'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

from Settings.TestData import TestData
from TestUnits.Test_main import Test_main

'''
 @brief Subclass due to limitations with the test class.
'''
class Test_DroneMasterSubClass(Test_main, TestData):
	def __init__(self):
		'''CONSTRUCTOR'''
		self.InitDroneMasterTest()

	def InitDroneMasterTest(self):
		'''
		 @brief Init the test variables
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src import DroneMaster
		self.Settings 		= Settings
		self.DroneMaster 	= DroneMaster
		##################

	def GetConfiguredSettings(self, folder, left_frames, right_frames):
		'''
		 @brief Get configured settings with correct input images/videos

		 @param folder
		 @param left_frames (tuple as (left_fn_frame, left_sl_frame))
		 @param right_frames (tuple as (right_fn_frame, right_sl_frame))

		 @return settings_inst
		'''
		import sys
		
		if 'camera' in sys.argv:
			self.source_type = 'CAMERA'
		elif 'image' in sys.argv:
			self.source_type = 'IMAGE'

		left_fn_frames 		= []
		left_fn_sl_frames 	= []
		right_fn_frames 	= []
		right_fn_sl_frames 	= []
		for i in range(len(left_frames)):
			left_fn_frames.append(left_frames[i][0])
			left_fn_sl_frames.append(left_frames[i][1])
			right_fn_frames.append(right_frames[i][0])
			right_fn_sl_frames.append(right_frames[i][1])
		settings_inst = self.Settings.Settings()
		settings_inst.ChangeSetting('BASIC', 'source_type', self.source_type)
		settings_inst.ChangeSetting('REAL_TIME_PLOT', 'real_time_plot_on', self.real_time_plot_on)
		settings_inst.ChangeSetting('REAL_TIME_PLOT', 'use_matplotlib', self.real_time_plot_matplotlib)
		settings_inst.ChangeSetting('CAMERA', 'n_frames', self.n_camera_frames)
		settings_inst.ChangeSetting('DATABASE', 'store_process_data', self.store_process_data)
		settings_inst.ChangeSetting('DATABASE', 'store_frames_as_video', self.store_frames_as_video)
		settings_inst.ChangeSetting('DATABASE', 'store_frames_as_images', self.store_frames_as_images)
		settings_inst.ChangeSetting('DATABASE', 'draw_heading', self.draw_heading)
		settings_inst.ChangeSetting('DATABASE', 'sub_output_folder', self.sub_output_folder)
		settings_inst.ChangeSetting('DATABASE', 'table_name', self.table_name)
		settings_inst.ChangeSetting('DATABASE', 'print_progress', self.print_progress)
		settings_inst.ChangeSetting('IMAGE', 'input_folder', folder)
		settings_inst.ChangeSetting('IMAGE', 'left_images', left_fn_frames)			# Master is to the left
		settings_inst.ChangeSetting('IMAGE', 'left_sl_images', left_fn_sl_frames)			
		settings_inst.ChangeSetting('IMAGE', 'right_images', right_fn_frames)			# Slave is to the right
		settings_inst.ChangeSetting('IMAGE', 'right_sl_images', right_fn_sl_frames)		
		settings_inst.ChangeSetting('USER_INPUT', 'automatic_mode', self.automatic_mode)
		settings_inst.ChangeSetting('CAMERA', 'manual_triggering', self.manual_triggering)
		settings_inst.ChangeSetting('TCP', 'master_ip', self.master_ip)
		if self.CheckAllTests():
			settings_inst.ChangeSetting('USER_INPUT', 'automatic_mode', True)
			settings_inst.ChangeSetting('CAMERA', 'manual_triggering', False)
			settings_inst.ChangeSetting('CAMERA', 'n_frames', 2)
		if self.source_type != 'CAMERA':
			settings_inst.ChangeSetting('USER_INPUT', 'automatic_mode', True)
			settings_inst.ChangeSetting('CAMERA', 'manual_triggering', False)
		if self.source_type == 'CAMERA' and settings_inst.GetSettings('CAMERA', 'manual_triggering'):
			settings_inst.ChangeSetting('CAMERA', 'n_frames', -1)
		return settings_inst

	def TestDroneMasterSlave(self):
		'''
		 @brief Test function for DroneMaster (With slave in thread)
		'''
		print 'Testing DroneMaster with DroneSlave in separate thread'

		from Test_DroneSlaveSubClass import Test_DroneSlaveSubClass
		from src.bin.tools import RunThread

		test_DroneSlave = Test_DroneSlaveSubClass()
		RunThread(test_DroneSlave.TestDroneSlave)
		self.TestDroneMaster()

	def TestDroneMaster(self):
		'''
		 @brief Test function for DroneMaster (no thread)
		'''
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				settings_inst 	= self.GetConfiguredSettings(folder, left_frames, right_frames)
				droneMaster 	= self.DroneMaster.DroneMaster(settings_inst=settings_inst)
				droneMaster.InitializeMaster()
				droneMaster.RunMaster()
				droneMaster.CloseMaster()
				if settings_inst.GetSettings('BASIC', 'source_type') == 'CAMERA':
					break