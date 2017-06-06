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
 @brief Test unit for PtGreyJordens
'''
class Test_PtGreyJordens(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DroneVision.DroneVision_src.hardware.PtGrey import PtGreyJordens
		self.Settings 		= Settings
		self.PtGreyJordens 	= PtGreyJordens
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass
		
	def test_PtGreyCaptureFrame(self):
		'''
		 @brief Test PtGreyJordens
		'''
		from src.DroneVision.DroneVision_src.hardware.imageTools import MatplotShow, RealTimePlot
		from src.DroneVision.DroneVision_src.hardware.PyQtImage import PyQtImage
		from Settings.Exceptions import PtGreyError
		from src.bin.UserInput.UserInput import UserInput
		import warnings
		from getpass import getpass

		settings 	= self.Settings.Settings()
		ptgrey 		= self.PtGreyJordens.PtGreyJordens(settings.GetSettings('CAMERA', 'camera_triggerPin'), \
			settings.GetSettings('CAMERA', 'ptg_triggerPin'), \
			False, \
			settings.GetSettings('CAMERA', 'ptg_recv_frame_timeout'))

		print '\n\n'
		#print 'FORMAT7: {0}\n\n'.format(ptgrey.SetFormat7Configuration(mode=ptgrey.GetFc2().MODE_0, pixel_format=ptgrey.GetFc2().PIXEL_FORMAT_RGB8))
		#print 'VIDEO AND FRAME RATE: {0}\n\n'.format(ptgrey.SetVideoModeAndFrameRate(video_mode=ptgrey.GetFc2().VIDEOMODE_FORMAT7)) # Unknown error when setting video mode and frame rate
		print 'FORMAT7: {0}\n\n'.format(ptgrey.SetFormat7Configuration(mode=ptgrey.GetFc2().MODE_0, pixel_format=ptgrey.GetFc2().PIXEL_FORMAT_RAW8))
		print 'CONFIGURATIONS: {0}\n\n'.format(ptgrey.SetConfiguration(num_buffers=3))

		ptgrey.StartCapture()

		print "######## SET MANUAL ##########\n\n"

		''' FOR RGB8 FRAME SETTINGS '''
		# print 'FORMAT7 (warning): {0}\n\n'.format(ptgrey.SetFormat7Configuration()) # Expect warning
		# print 'VIDEO AND FRAME RATE (warning): {0}\n\n'.format(ptgrey.SetVideoModeAndFrameRate()) #Expect warning
		# print 'CONFIGURATIONS (warning): {0}\n\n'.format(ptgrey.SetConfiguration()) # Expect warning
		# print 'FRAME RATE: {0}\n\n'.format(ptgrey.SetFrameRate(32.0))
		# print 'SET GAIN: {0}\n\n'.format(ptgrey.SetGain(7.3, auto=False))
		# print 'SET SHUTTER: {0}\n\n'.format(ptgrey.SetShutter(300.52, auto=False))
		# print 'SET BRIGHTNESS: {0}\n\n'.format(ptgrey.SetBrightness(0.5))
		# print 'SET AUTO EXPOSURE: {0}\n\n'.format(ptgrey.SetAutoExposure(1.34, auto=False))
		# #print 'SET SHARPNESS: {0}\n\n'.format(ptgrey.SetSharpness(7.3, auto=False)) # Unkown api error
		# print 'SET GAMMA: {0}\n\n'.format(ptgrey.SetGamma(1.5, auto=False))
		# print 'SET WHITE BALANCE: {0}\n\n'.format(ptgrey.SetWhiteBalance(1536, 0, auto=False))

		''' FOR RAW8 FRAME SETTINGS '''
		print 'FORMAT7 (warning): {0}\n\n'.format(ptgrey.SetFormat7Configuration()) # Expect warning
		print 'VIDEO AND FRAME RATE (warning): {0}\n\n'.format(ptgrey.SetVideoModeAndFrameRate()) #Expect warning
		print 'CONFIGURATIONS (warning): {0}\n\n'.format(ptgrey.SetConfiguration()) # Expect warning
		print 'FRAME RATE: {0}\n\n'.format(ptgrey.SetFrameRate(32.0))
		print 'SET GAIN: {0}\n\n'.format(ptgrey.SetGain(28.09, auto=False))
		print 'SET SHUTTER: {0}\n\n'.format(ptgrey.SetShutter(109.61, auto=False))
		print 'SET BRIGHTNESS: {0}\n\n'.format(ptgrey.SetBrightness(3.0))
		print 'SET AUTO EXPOSURE: {0}\n\n'.format(ptgrey.SetAutoExposure(0.923, auto=False))
		#print 'SET SHARPNESS: {0}\n\n'.format(ptgrey.SetSharpness(28.1, auto=False)) # Unkown api error
		print 'SET GAMMA: {0}\n\n'.format(ptgrey.SetGamma(1.5, auto=False))
		print 'SET WHITE BALANCE: {0}\n\n'.format(ptgrey.SetWhiteBalance(1536, 0, auto=False))

		if self.ptgrey_grab_infinite:
			#realTimePlot = RealTimePlot(interactive_mode=True)
			realTimePlot = PyQtImage(True)
			settings.ChangeSetting('USER_INPUT', 'automatic_mode', True)
			userInput = UserInput(settings.GetSettings('USER_INPUT'))
		while True:
			try:
				frame = ptgrey.CaptureFrame()
			except PtGreyError, err:
				warnings.simplefilter('always')
				warnings.warn(str(err), Warning)
				warnings.simplefilter('default')
				ptgrey.RestartCapture()
				continue

			print 'Captured frame with shape: {0} and type: {1}'.format(frame.shape, frame.dtype)
			matplotlist = [('Frame', frame)]
			print "######## SET AUTO ##########\n\n"
			print 'SET GAIN: {0}\n\n'.format(ptgrey.SetGain(auto=True))
			print 'SET SHUTTER: {0}\n\n'.format(ptgrey.SetShutter(auto=True))
			print 'SET BRIGHTNESS: {0}\n\n'.format(ptgrey.SetBrightness(3.0))
			print 'SET AUTO EXPOSURE: {0}\n\n'.format(ptgrey.SetAutoExposure(auto=True))
			#print 'SET SHARPNESS: {0}\n\n'.format(ptgrey.SetSharpness(auto=True)) # Unkown api error
			print 'SET GAMMA: {0}\n\n'.format(ptgrey.SetGamma(auto=True))
			print 'SET WHITE BALANCE: {0}\n\n'.format(ptgrey.SetWhiteBalance(auto=True))

			if not(self.CheckAllTests()):
				if self.ptgrey_grab_infinite:
					#realTimePlot(matplotlist)
					realTimePlot.UpdatePQImages(matplotlist)
					if userInput.CheckTerminated():
						break
				else:
					MatplotShow(matplotlist, save_fig=self.save_figs, save_fig_only=self.save_figs_only)
					if settings.GetSettings('CAMERA', 'manual_triggering'):
						q = getpass('Enter "q" to quit, or nothing to continue..')
						if q == 'q':
							break
			else:
				break

		ptgrey.DisconnectCamera()
