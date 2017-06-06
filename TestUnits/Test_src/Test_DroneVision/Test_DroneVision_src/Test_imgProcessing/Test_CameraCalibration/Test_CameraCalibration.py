'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

################### UNIT TEST ########################
import unittest

from Settings.TestData import TestData
from TestUnits.Test_main import Test_main
'''
 @brief Test unit for CameraCalibration
'''
class Test_CameraCalibration(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.imgProcessing.CameraCalibration import CameraCalibration
		self.Settings 			= Settings
		self.CameraCalibration 	= CameraCalibration
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_CameraCalibration(self):
		'''
		 @brief Test camera calibration unit by undistorting chess board images.
		'''
		from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import PyrDown, GetShape
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, MatplotShow, RealTimePlot

		settings_inst = self.Settings.Settings()
		settings_inst.ChangeSetting('BASIC', 'reset_calibration', True)
		settings_inst.ChangeSetting('CALIB', 'calib_show_imgs', not(self.CheckAllTests()))
		settings_inst.ChangeSetting('CALIB', 'calib_print_process', True)
		settings_inst.ChangeSetting('CALIB', 'calib_show_imgs', True)
		use_PyQt = False
		realTimePlot = RealTimePlot(interactive_mode=False)
		#realTimePlot = None
		left_calib 	= self.CameraCalibration.CameraCalibration(settings_inst.GetSettings('CALIB'), settings_inst.GetSettings('CALIB', 'calib_img_folder_left_cam'), settings_inst.GetSettings('CALIB', 'calib_save_fname_left_cam'), settings_inst.GetSettings('BASIC', 'reset_calibration'), plot_figure=realTimePlot, use_PyQt=use_PyQt)
		right_calib = self.CameraCalibration.CameraCalibration(settings_inst.GetSettings('CALIB'), settings_inst.GetSettings('CALIB', 'calib_img_folder_right_cam'), settings_inst.GetSettings('CALIB', 'calib_save_fname_right_cam'), settings_inst.GetSettings('BASIC', 'reset_calibration'), plot_figure=realTimePlot, use_PyQt=use_PyQt)
		calibs = [(left_calib, 'Left: '), (right_calib, 'Right: ')]

		touple_frames = []
		for i in range(len(calibs)):
			calib, calib_name = calibs[i]
			calib.CalibrateCameraDistortion()
			calib.ShowTestCalibImage()

			test_img_fname 			= calib.GetDistorionCalibImages()[0] 
			test_img 				= GetImage(test_img_fname)
			test_img 				= PyrDown(test_img, settings_inst.GetSettings('CV', 'default_downsampling_divisor'), settings_inst.GetSettings('CV', 'desired_frame_shape'))
			
			headline 				= calib_name + 'before shape {0}'.format(test_img.shape)
			touple_frames.append((headline, test_img))
			
			test_und_img 			= calib.Undistort(test_img)
			headline 				= calib_name + 'After undistort shape {0}'.format(test_und_img.shape)
			touple_frames.append((headline, test_und_img))
		
		if not(self.CheckAllTests()):
			MatplotShow(touple_frames, test_img_fname+'_Camera_calibration_test', save_fig=self.save_figs, save_fig_only=self.save_figs_only)

