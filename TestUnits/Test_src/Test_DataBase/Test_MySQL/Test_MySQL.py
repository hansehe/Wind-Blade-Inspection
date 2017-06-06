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
 @brief Test unit for MySQL
'''
class Test_MySQL(unittest.TestCase, Test_main, TestData):

	def setUp(self):
		'''
		 @brief Give all setups to the unit test.
		'''
		self.SetAllKey()
		self.InitTestData()
		#### IMPORTS #####
		from Settings import Settings
		from src.DataBase.MySQL import MySQL
		self.Settings 	= Settings
		self.MySQL 		= MySQL
		##################

	def tearDown(self):
		'''
		 @brief Give all tear down steps. 
		 	Is runned even if the test failed.
		'''
		pass

	def test_MySQL(self):
		'''
		 @brief Test function for MySQL
		'''
		import numpy as np
		import time
		from datetime import datetime
		mysql = self.MySQL.MySQL(self.username, self.password, self.database, self.host)
		databases = mysql.GetSQLDatabases()
		print databases

		frame = np.ones((16, 24), dtype=np.uint8)
		table_name = 'Test_mysql'
		unix_time  = datetime.utcnow().strftime('%Y_%m_%d__%H_%M_%S')
		table_name += "_"+unix_time
		if mysql.CheckSQLTableExist(table_name):
			mysql.DropSQLTable(table_name)

		process_frames 	= {'frame_1': frame, 'frame_2': frame}
		process_data 	= {'distance': 4424.99, 'rho': 12.055, 'theta': 22.44}

		primary_key = 'timestamp'
		columns = {primary_key: 'TIMESTAMP(6)', 'longitude': 'FLOAT', 'langitude': 'FLOAT', 'latitude': 'FLOAT'}
		for key in process_data:
			columns[key] = mysql.FindSQLValueType(process_data[key])
		if not(mysql.CheckSQLTableExist(table_name)):
			mysql.CreateSQLTable(table_name, columns, primary_key)
		for key in columns:
			if not(mysql.CheckSQLColumnExist(table_name, key)):
				mysql.AddColumnToSQLTable(table_name, (key, columns[key]))

		n_tables = 4
		for i in range(n_tables):
			#unix_time 		= int(time.time())
			unix_time 		= datetime.utcnow().strftime('%Y-%m-%d %H-%M-%S.%f')
			#unix_time 		= datetime.utcnow()
			print unix_time
			#unix_time 		= time.strftime("%Y-%m-%d %H-%M-%S", time.gmtime())
			gps_position 	= (0.0,0.0,0.0)
			data = {'timestamp': unix_time, 'longitude': gps_position[0], 'langitude': gps_position[1], 'latitude':gps_position[2]}
			for key in process_data:
				data[key] = mysql.ConvertPythonToSQLValue(process_data[key])
			mysql.InsertDataIntoSQLTable(table_name, data)
			time.sleep(0.2)
		
		columns  = mysql.GetSQLColumns(table_name)
		print 'Columns: ', columns
		for i in range(len(columns)):
			column_key = columns[i][0]
			column_type = columns[i][1]
			mysql.SelectDataFromSQLTable(table_name, column_key, n_rows=2, row_number=1)
			sql_rows = [None]
			while len(sql_rows) > 0:
				sql_rows = mysql.FetchSelectedSQLData(0)
				print '\n\n\n'
				print sql_rows
				if sql_rows == None:
					break
				for sql_row in sql_rows:
					value = mysql.ConvertSQLValueToPython(sql_row[0], column_type)
					if not(column_key == 'timestamp'):
						value += 5
						value *= 10
					print 'SQL COLUMN ({0}): '.format(column_key), value

		mysql.CloseSQL()