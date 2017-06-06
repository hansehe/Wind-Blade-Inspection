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
class Test_MasterSubClass(Test_main, TestData):
	def __init__(self):
		'''CONSTRUCTOR'''
		self.InitMasterTest()

	def InitMasterTest(self):
		'''
		 @brief Init the test variables
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.MasterSlave import Master
		from TestUnits.Test_src.Test_MasterSlave.Test_SlaveSubClass import Test_SlaveSubClass
		self.Settings 		= Settings
		self.Master 		= Master
		self.test_ObjSlave	= Test_SlaveSubClass()
		##################

	def TestConnect(self):
	    '''
	     @brief Test connection setup between Master and Slave.
	    '''
	    print 'Testing connect to slave'
	    self.objMaster.Connect()

	def TestStayingAlive(self, timeout):
	    '''
	     @brief Test Slave and Master just waiting.
	    '''
	    import time
	    print 'Testing staying alive'
	    time.sleep(timeout)

	def TestRequestFrame(self):
	    '''
	     @brief Test frame request.
	    '''
	    import numpy as np
	    print 'Testing getFrame slave request'
	    frame_content, valid, error = self.objMaster.RequestFrame()
	    original_frame, original_sl_frame, frame_un, delta_frame, point_kp, blob_desc = frame_content

	    self.TestMatchingFrameRequest(original_frame)
	    self.TestMatchingFrameRequest(original_sl_frame)
	    self.TestMatchingFrameRequest(frame_un)
	    self.TestMatchingFrameRequest(delta_frame)

	def TestRequestPointList(self):
		'''
		 @brief Test point list request.
		'''
		import numpy as np
		print 'Testing getPointList slave request'
		und_shape, point_list, blob_desc, valid, error = self.objMaster.RequestPointList()

 	def TestMatchingFrameRequest(self, np_frame):
		'''
		 @brief Test matching frames 

		 @param np_frame
		'''
		if not(np_frame.all() == self.test_ObjSlave.TestGetFrame().all()):
			raise Exception('Frame request failed: frame mismatch')

	def TestRequestRestart(self):
		'''
		 @brief Test restart request.
		'''
		print 'Testing restart slave request'
		self.objMaster.RequestRestart()
		self.TestConnect() #Test reconnect

	def TestRequestCameraRestart(self):
		'''
		 @brief Test camera restart request
		'''
		print 'Testing camera restart slave request'
		self.objMaster.RequestRestartPtGrey()

	def TestRequestStop(self):
		'''
		 @brief Test stop request.
		'''
		print 'Testing stop slave request'
		self.objMaster.RequestStop()

	def TestGeneralRequests(self):
		'''
		 @brief Test general requests.
		'''
		self.objMaster.RequestSetTimestamp(10)
		self.objMaster.RequestCVCalibration(True, True)
		if not(self.objMaster.RequestSlaveReady()):
			raise Exception('Error receiving ready (True) from slave')
		self.objMaster.RequestFrameProcessingOnSlave()

	def TestMasterSlave(self):
		'''
		 @brief Unit test for testing master and slave in a single test.
		'''
		from src.bin.tools import RunThread

		RunThread(self.test_ObjSlave.TestSlave)

		self.TestMaster()

	def TestMaster(self):
		'''
		 @brief Unit test for testing master and slave in separate tests.
		'''
		print 'Master test initiated'

		settings_inst 	= self.Settings.Settings()
		self.objMaster  = self.Master.Master(settings_inst.GetSettings('TCP'))

		self.TestConnect()
		self.TestGeneralRequests()
		self.TestRequestFrame()
		self.TestRequestPointList()
		self.TestRequestCameraRestart()

		self.TestStayingAlive(settings_inst.GetSettings('TCP', 'tcp_timeout')*0.3)
		#self.TestRequestRestart() # Must be run independently (without thread)
		self.TestRequestStop()

		print 'Successfull Master/Slave unit test!'