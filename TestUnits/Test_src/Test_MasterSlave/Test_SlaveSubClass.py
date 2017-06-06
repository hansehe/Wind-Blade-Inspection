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
class Test_SlaveSubClass(Test_main, TestData):
	def __init__(self):
		'''CONSTRUCTOR'''
		self.InitSlaveTest()

	def InitSlaveTest(self):
		'''
		 @brief Init the test variables
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.MasterSlave import Slave
		self.Settings 	= Settings
		self.Slave 		= Slave
		##################

	def TestGetFrame(self):
		'''
		 @brief Get a test frame

		 @return frame (numpy array)
		'''
		import numpy as np # Only needed for the unit test
		return np.ones((124, 124), dtype=np.uint8)*10 #Create uint8 numpy array (frame)

	def SetProcessNewFrameFlag(self):
		'''
		 @brief Test function for SetProcessNewFrameFlag(). 
		 Actually a function from DroneSlave during normal operation.
		'''
		print 'processing new frame..'

	def GetFramePayload(self):
		'''
		 @brief Test function for GetFramePayload(). 
		 Actually a function from DroneSlave during normal operation.
		'''
		import numpy as np
		original_frame 		= self.TestGetFrame()
		original_sl_frame 	= self.TestGetFrame()
		frame_un 			= self.TestGetFrame()
		delta_frame 		= self.TestGetFrame()
		point_list 			= []
		blob_desc 			= np.zeros(0)
		frame_content 		= (original_frame, original_sl_frame, frame_un, delta_frame, point_list, blob_desc)
		return frame_content, None

	def RestartCamera(self):
		'''
		 @brief simulating camera restart
		'''
		print 'restarting camera..'

	def SetTimestamp(self, timestamp):
		'''
		 @brief Test function for SetTimestamp(). 
			Actually a function from DroneSlave during normal operation.

		 @param timestamp
		'''
		print 'Timestamp received: ', timestamp

	def SetStoreDBFlag(self):
		'''
		 @brief Test SetStoreDBFlag()
		'''
		print 'Setting store database flag'

	def CalibrateCV(self, cam_calbib, blob_calib):
		'''
		 @brief Test function for CalibrateCV(). 
			Actually a function from DroneSlave during normal operation.
		'''
		print 'Simulating CV calibration..'

	def GetSlaveReady(self):
		'''
		 @brief Test function for GetSlaveReady(). 
			Actually a function from DroneSlave during normal operation.
		'''
		print 'Simulating slave ready calibration..'
		return True

	def TestSlave(self):
	    '''
	     @brief Unit test.
	    '''
	    print 'Slave test initiated'
	    settings_inst 	= self.Settings.Settings()
	    slave 			= self.Slave.Slave(settings_inst.GetSettings('TCP'), self)
	    slave.Connect()
	    while not(slave.GetTerminate()):
	        pass
	    print 'Slave test terminated'