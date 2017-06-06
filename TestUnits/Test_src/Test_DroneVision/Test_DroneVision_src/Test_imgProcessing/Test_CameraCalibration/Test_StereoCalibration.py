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
 @brief Test unit for StereoCalibration
'''
class Test_StereoCalibration(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.imgProcessing.CameraCalibration import StereoCalibration
		self.Settings 			= Settings
		self.StereoCalibration 	= StereoCalibration
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_StereoCalibration(self):
		'''
		 @brief Test function for the stereo calibration unit
		'''
		from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import PyrDown
		from src.DroneVision.DroneVision_src.hardware.imageTools import GetImage, MatplotShow

		settings_inst = self.Settings.Settings()
		settings_inst.ChangeSetting('BASIC', 'reset_calibration', True)
		settings_inst.ChangeSetting('CALIB', 'calib_show_imgs', not(self.CheckAllTests()))
		settings_inst.ChangeSetting('CALIB', 'calib_print_process', True)
		settings_inst.ChangeSetting('CALIB', 'calib_show_imgs', False)
		stereo_calib = self.StereoCalibration.StereoCalibration(True, settings_inst.GetSettings('CALIB'), settings_inst.GetSettings('BASIC', 'reset_calibration'))
		stereo_calib.CalibrateStereoVisionSystem()

		# Try reloading save parameters
		settings_inst.ChangeSetting('BASIC', 'reset_calibration', False)
		stereo_calib = self.StereoCalibration.StereoCalibration(True, settings_inst.GetSettings('CALIB'), settings_inst.GetSettings('BASIC', 'reset_calibration'))
		stereo_calib.CalibrateStereoVisionSystem()
		calibs = [(stereo_calib.GetLeftCameraCalibrationInstance(), 'Left: '), (stereo_calib.GetRightCameraCalibrationInstance(), 'Right: ')]

		touple_frames = []
		for i in range(len(calibs)):
			calib, calib_name = calibs[i]

			test_img_fname 			= calib.GetDistorionCalibImages()[0]
			test_img 				= GetImage(test_img_fname)
			test_img 				= PyrDown(test_img, settings_inst.GetSettings('CV', 'default_downsampling_divisor'), settings_inst.GetSettings('CV', 'desired_frame_shape'))
			
			headline 				= calib_name + 'before shape {0}'.format(test_img.shape)
			touple_frames.append((headline, test_img))
			
			test_und_img 			= stereo_calib.Undistort(test_img)
			headline 				= calib_name + 'stereo und shape {0}'.format(test_und_img.shape)
			touple_frames.append((headline, test_und_img))
		
		if not(self.CheckAllTests()):
			MatplotShow(touple_frames, test_img_fname+'_Stereo_calibration_test', save_fig=self.save_figs, save_fig_only=self.save_figs_only)