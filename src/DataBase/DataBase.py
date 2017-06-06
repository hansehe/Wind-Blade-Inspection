'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''
import time, json
import numpy as np
from datetime import datetime

from src.bin.tools import GetTimestamp
from MySQL.MySQL import MySQL
from GPS.GPS import GPS
from LogTool.LogTool import LogTool
from FrameRecorder.FrameRecorder import FrameRecorder

'''
 @brief DataBase class for storing data.

 @param subclass DroneMaster or DroneSlave class (self)
 @param settings_inst (DATABASE settings)
 @param default_output_folder (default output folder (default=''))
'''
class DataBase(MySQL, GPS, LogTool, FrameRecorder):
	def __init__(self, subclass, settings_inst, default_output_folder=''):
		'''CONSTRUCTOR'''
		self.__output_folder 	= default_output_folder
		self.__table_created 	= False
		self.__primary_key 		= ('timestamp', 'TIMESTAMP(6)')
		self.__database 		= settings_inst.GetSettings('database')
		self.__table_name 		= settings_inst.GetSettings('table_name')
		MySQL.__init__(self, settings_inst.GetSettings('username'), settings_inst.GetSettings('password'), database=self.__database)
		LogTool.__init__(self, subclass, printToScreen=settings_inst.GetSettings('print_progress'))
		GPS.__init__(self)
		FrameRecorder.__init__(self, settings_inst.GetSettings('store_frames_video_fps'), settings_inst.GetSettings('store_frames_as_video'), settings_inst.GetSettings('store_frames_as_images'))

	def GetDatabaseName(self):
		'''
		 @brief Get database name for current session

		 @return database
		'''
		return self.__database

	def SetDataBaseOutputFolder(self, output_folder):
		'''
		 @brief Set output folder

		 @param output_folder
		'''
		self.__output_folder = output_folder
		self.SetLogFilename(self.__output_folder + 'Log.txt')
		self.InitFrameRecorder(self.__output_folder)

	def GetDatabaseOutputFolder(self):
		'''
		 @brief Get the database output folder

		 @return output_folder
		'''
		return self.__output_folder

	def SetDatabaseTableName(self, timestamp, master=False, wait_for_user=True):
		'''
		 @brief Initialize database by setting the table name with a timestamp. 

		 @param timestamp (datetime, string)
		 @param master (True/False)
		 @param wait_for_user (True/False)
		'''
		self.PrintStarting(master=master, wait_for_user=wait_for_user)
		if len(self.__table_name) > 0:
			self.__table_name = self.__table_name + '_' + timestamp
		else:
			self.__table_name = timestamp

	def GetDatabaseTable(self):
		'''
		 @brief Get table name for current session

		 @return table_name
		'''
		return self.__table_name

	def CreateNewTable(self, dict_list):
		'''
		 @brief Create new table in database with correct columns and values

		 @param dict_list (list of data dictionaries to store in database)
		'''
		columns = {self.__primary_key[0]: self.__primary_key[1]}
		for data_dict in dict_list:
			for key in data_dict:
				columns[key] = self.FindSQLValueType(data_dict[key])
		if not(self.CheckSQLTableExist(self.__table_name)):
			self.CreateSQLTable(self.__table_name, columns, self.__primary_key[0])
		else:
			for key in columns:
				if not(self.CheckSQLColumnExist(self.__table_name, key)):
					self.AddColumnToSQLTable(self.__table_name, (key, columns[key]))
		self.__table_created = True

	def InsertToDatabase(self, dict_list):
		'''
		 @brief Insert data to database.

		 @param dict_list (list of data dictionaries to store in database)
		'''
		if not(self.__table_created):
			self.CreateNewTable(dict_list)
		unix_time 	= GetTimestamp(fractional=True)
		data 		= {self.__primary_key[0]: unix_time}
		for data_dict in dict_list:
			for key in data_dict:
				if not(np.isnan(data_dict[key])) and not(data_dict[key] == None):
					data[key] = data_dict[key]
		try:
			self.InsertDataIntoSQLTable(self.__table_name, data)
		except: # Check if columns are missing, and try a second time
			for key in data:
				if not(self.CheckSQLColumnExist(self.__table_name, key)):
					self.AddColumnToSQLTable(self.__table_name, (key, self.FindSQLValueType(data[key])))
			self.InsertDataIntoSQLTable(self.__table_name, data)

	def RecordData(self, record_frames=True, insert_to_database=True, print_progress=False, points3D=[]):
		'''
		 @brief Record data and frames to database. The frames are also recorded as video and images.
		
		 @param record_frames (True/False for recording frames (default=True))
		 @param insert_to_database (True/False for inserting data to database (default=True))
		 @param print_progress (Default=False)
		 @param points3D (for printing 3D points (default=[]))
		'''
		if print_progress:
			self.PrintProgress(points3D=points3D)
		if record_frames:
			self.RecordProcessFrames()
		self.ResetProcessFrames() # Reset process frames to lighten memory
		if insert_to_database:
			dict_list = [self.GetGPSPosition(), self.GetProcessData()]
			self.InsertToDatabase(dict_list)
		self.ResetProcessData()

	def CloseDataBase(self):
		'''
		 @brief Close DataBase
		'''
		MySQL.__del__(self)
		LogTool.__del__(self)
		FrameRecorder.__del__(self)

	def __del__(self):
		'''DESTRUCTOR'''
		self.CloseDataBase()

