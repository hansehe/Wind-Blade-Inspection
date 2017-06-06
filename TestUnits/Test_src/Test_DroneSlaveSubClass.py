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
class Test_DroneSlaveSubClass(Test_main, TestData):
	def __init__(self):
		'''CONSTRUCTOR'''
		self.InitDroneSlaveTest()

	def InitDroneSlaveTest(self):
		'''
		 @brief Init the test variables
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src import DroneSlave
		self.Settings 	= Settings
		self.DroneSlave = DroneSlave
		##################

	def GetConfiguredSettings(self, folder, left_frames, right_frames):
		'''
		 @brief Get configured settings with correct input images/videos

		 @param folder
		 @param left_frames (tuple as (left_fn_frame, left_sl_frame))
		 @param right_frames (tuple as (right_fn_frame, right_sl_frame))

		 @return settings_inst
		'''
		from TestUnits.Test_src.Test_DroneMasterSubClass import Test_DroneMasterSubClass
		test_DroneMaster = Test_DroneMasterSubClass()
		return test_DroneMaster.GetConfiguredSettings(folder, left_frames, right_frames) # Follow settings syntax from DroneMaster test

	def TestDroneSlave(self, print_progress=False):
		'''
		 @brief Test function for DroneSlave
		'''
		for folder, left_frames, right_frames, actual_distances, baselines, use_set in self.GetFrameSets():
			if use_set:
				settings_inst 	= self.GetConfiguredSettings(folder, left_frames, right_frames)
				settings_inst.ChangeSetting('DATABASE', 'print_progress', print_progress)
				droneSlave 		= self.DroneSlave.DroneSlave(settings_inst=settings_inst)
				droneSlave.InitializeSlave()
				droneSlave.RunSlave()
				droneSlave.CloseSlave()
				if settings_inst.GetSettings('BASIC', 'source_type') == 'CAMERA':
					break