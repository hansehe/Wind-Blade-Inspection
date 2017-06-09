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
 @brief Test unit for CameraLink
'''
class Test_CameraLink(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.hardware import CameraLink
		self.Settings 	= Settings
		self.CameraLink = CameraLink
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_CameraLink(self):
		'''
		 @brief Test Ptg camera link by receiving frames, and saving them. 
		'''
		if not(self.CheckAllTests()): # Trigger test covers the same manually
			return

		import timeit
		from src.DroneVision.DroneVision_src.hardware.imageTools import WriteImage, GetImage
		from src.bin.tools import GetTimestamp, CheckDir

		##### SETUP FOLDERS ######
		settings_inst 		= self.Settings.Settings()
		settings_inst.ChangeSetting('CAMERA', 'manual_triggering', False)
		test_frame_folder 	= settings_inst.GetSettings('DATABASE', 'output_folder') + '/TEST/' + GetTimestamp() + '/'
		test_frame_folder 	+= 'left_test_camera/'
		CheckDir(test_frame_folder)
		##########################

		cameralink = self.CameraLink.CameraLink(True, settings_inst.GetSettings('CAMERA'), settings_inst.GetSettings('LASER'))
		start_time = timeit.default_timer()
		raise_error = False
		i = 0
		try:
			while timeit.default_timer() - start_time < 5:
				frame, sl_frame = cameralink.GetFrame()
				print frame.shape
				fps, width, height  = cameralink.GetFrameProperties()
				total_frames 		= cameralink.GetTotalFrames()
				frame_n 			= cameralink.GetFrameNumber()
				print 'GET: ', fps, width, height, total_frames, frame_n
				WriteImage(frame, test_frame_folder + 'frame_' + str(frame_n))
				frame = GetImage(test_frame_folder + 'frame_' + str(frame_n) + '.tif')
				i += 1
		except:
			raise_error = True
		cameralink.StopCamera()
		if raise_error:
			raise

	def test_TriggerCamera(self):
		'''
		 @brief Test manual camera triggering
		'''
		'''
		 @brief Test Ptg camera link by receiving frames, and saving them. 
		'''
		if self.CheckAllTests(): #Cannot test while auto testing
			return

		import sys
		from getpass import getpass
		from src.DroneVision.DroneVision_src.hardware.imageTools import WriteImage, GetImage, MatplotShow
		from src.DroneVision.DroneVision_src.imgProcessing.featureDetection.PointDetection import PointDetection
		from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import CheckGrayScale, FilterByColor
		from src.bin.tools import GetTimestamp, CheckDir
		from src.bin.UserInput.UserInput import UserInput

		this_master = True
		if self.slave_key in sys.argv:
			this_master = False

		##### SETUP FOLDERS ######
		settings_inst 		= self.Settings.Settings()
		settings_inst.ChangeSetting('CAMERA', 'manual_triggering', self.manual_triggering)
		settings_inst.ChangeSetting('CAMERA', 'ptg_recv_frame_timeout', self.camera_capture_timeout)
		test_frame_folder 	= settings_inst.GetSettings('DATABASE', 'output_folder') + '/TEST_TRIGGER/' + GetTimestamp() + '/'
		if this_master:
			test_frame_folder 	+= 'left_test_camera/'
		else:
			test_frame_folder 	+= 'right_test_camera/'
		CheckDir(test_frame_folder)
		##########################

		pointDet 	= PointDetection.PointDetection(True, settings_inst.GetSettings())
		pointDet.CalibratePointDetection()
		cameralink = self.CameraLink.CameraLink(this_master, settings_inst.GetSettings('CAMERA'), settings_inst.GetSettings('LASER'))
		
		if this_master:
			s_title = 'Master Frames'
		else:
			s_title = 'Slave Frames'

		settings_inst.ChangeSetting('USER_INPUT', 'automatic_mode', self.manual_triggering)
		getpass('Press enter to start - hold q to quit..')
		userInput = UserInput(settings_inst.GetSettings('USER_INPUT'))
		print 'START CAMERA TRIGGER TEST'
		i = 0
		raise_error = False
		try:
			while (not(userInput.CheckTerminated())):
				title = s_title + ' {0}'.format(i)
				print 'Reading frame..'
				frame, sl_frame = cameralink.GetFrame()
				print 'FRAME SHAPE: ', frame.shape
				#green_mask, g_frame, g_sl_frame = pointDet.ComputeGreenMask(frame, sl_frame)
				green_mask, keypoints, descriptors, frame, sl_frame = pointDet.GetPointList(frame, sl_frame, draw=True, ignore_no_blobs_error=True)
				MatplotShow([('green_mask ({0})'.format(green_mask.shape), green_mask), ('frame ({0})'.format(frame.shape), frame), ('sl_frame ({0})'.format(sl_frame.shape), sl_frame)], title, save_fig=self.save_figs, save_fig_only=self.save_figs_only)
				#un_frame 	= pointDet.Undistort(frame)
				#un_sl_frame = pointDet.Undistort(sl_frame)
				#print 'Captured frame with shape {0}'.format(frame.shape)
				#MatplotShow([('Frame ({0})'.format(frame.shape), frame), ('UN FRAME ({0})'.format(un_frame.shape), un_frame), ('SL FRAME ({0})'.format(sl_frame.shape), sl_frame), ('UN SL FRAME ({0})'.format(un_sl_frame.shape), un_sl_frame)], title, save_fig=self.save_figs, save_fig_only=self.save_figs_only)
				#WriteImage(frame, test_frame_folder + 'frame_' + str(i))
				#WriteImage(sl_frame, test_frame_folder + 'frame_sl_' + str(i))
				i += 1
		except:
			print 'RAISED ERROR'
			raise_error = True
		userInput.ForceTermination()
		cameralink.StopCamera()
		if raise_error:
			raise

