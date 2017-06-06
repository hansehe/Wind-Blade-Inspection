'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master"s Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master"s Thesis - CV (Computer Vision)
'''
import MySQLdb as mdb
import numpy as np
import sys

'''
 @brief MySQL class.
 	Install mysql-server:
 		Download newest mysql package from dev.mysql.com/downloads (called mysql-apt-config_X.X.X-X_all.deb)
 		Install mysql config using downloaded package: sudo dpkg -i /PATH/version-specific-package-name.deb
 		Update system: sudo apt-get update
 		Install mysql server: sudo apt-get install mysql-server
 		Install mysqldb for python: sudo pip install MySQL-python
 	Start mysql server: sudo service mysql start 
 	Stop mysql server: sudo service mysql stop
 	Status mysql server: sudo service mysql status 
 	manual login to mysql: mysql -u root -p
 		Where root is username, and password where set when mysql was installed

 @param username
 @param password
 @param database (None for not created yet, (default=None))
 @param host (default="localhost")
'''
class MySQL():
	def __init__(self, username, password, database=None, host="localhost"):
		'''CONSTRUCTOR'''
		self.__sql_db_connected = False
		self.SetSQLConfig(username, password, database=database, host=host)

	def NumpyArrayToSQLBlob(self, np_array):
		'''
		 @brief Convert numpy array to sql blob data.
		 	Note: mysql is not built to handle large datas,
		 	so it is recommended to store large arrays as file and save filepath to mysql.

		 @param np_array

		 @return np_array_dump
		'''
		return np.ndarray.dumps(np_array)

	def SQLBlobToNumpyArray(self, np_array_str):
		'''
		 @brief Convert sql numpy blob to numpy array
		 	Note: mysql is not built to handle large datas,
		 	so it is recommended to store large arrays as file and save filepath to mysql.

		 @param np_array_str

		 @return np_array
		'''
		return np.loads(bytes(np_array_str))

	def FindSQLValueType(self, value):
		'''
		 @brief Find correct type for value 

		 @param value

		 @return db_type (as string to database)
		'''
		if isinstance(value, np.ndarray):
			db_type = 'BLOB'
		elif isinstance(value, float):
			db_type = 'FLOAT'
		elif isinstance(value, int):
			db_type = 'INT'
		elif isinstance(value, str):
			if len(value) == 1:
				db_type = 'CHAR(255)'
			elif len(value) > 255:
				db_type = 'TEXT'
			else:
				db_type = 'VARCHAR('+str(len(value)+50)+')'
		elif np.isnan(value):
			db_type = 'FLOAT'
		elif value == None:
			db_type = 'FLOAT'
		else:
			raise Exception('Unknown python to sql value type: {0}'.format(value))
		return db_type

	def ConvertSQLValueToPython(self, value, db_type):
		'''
		 @brief Convert sql value to python value 

		 @param value
		 @param db_type

		 @return python_value
		'''
		if 'BLOB' in db_type.upper():
			python_value = self.SQLBlobToNumpyArray(value)
		else:
			python_value = value
		return python_value

	def ConvertPythonToSQLValue(self, value):
		'''
		 @brief Convert sql value to python value 

		 @param value

		 @return sql_value
		'''
		db_type = self.FindSQLValueType(value)
		if 'BLOB' in db_type.upper():
			sql_value = self.NumpyArrayToSQLBlob(value)
		else:
			sql_value = value
		return sql_value

	def SelectDataFromSQLTable(self, table_name, column_key="*", n_rows=0, row_number=0):
		'''
		 @brief Select data from sql table.

		 @param table_name
		 @param column_key (default="*", meaning whole table is fetched)
		 @param n_rows (Number of rows to fetch, starting at row_number - n_rows, If n_rows is 0, then all rows are selected)
		 @param row_number (Last row number to select. Last row will be row N = row_number - n_rows, if row_number is 0, then all rows up to n_rows are selected)
		'''
		if n_rows <= 0:
			sql_command = "SELECT " + column_key + " FROM " + table_name
		else:
			sql_command = "SELECT " + column_key + " FROM " + table_name + " LIMIT " + str(row_number) + "," + str(n_rows)
		self.ExecuteSQLCommand(sql_command)

	def FetchSelectedSQLData(self, n_elements=0):
		'''
		 @brief Fetch selected data from sql table.

		 @param n_elements (Number of elements to fetch at this instance, 0= fetch all at once (default=0))

		 @return sql_rows (sql_rows = touple of touples, empty list is returned when no more rows are available)
		 	Returned as touple of touples, accessed as:
		 		for sql_row in sql_rows:
		 			print sql_row
		'''
		if n_elements < 0:
			raise ValueError('Invalid negative value for fetching sql data')
		if self.GetSQLConnected():
			if n_elements == 0:
				sql_rows = self.__sql_cursor.fetchall()
			elif n_elements == 1:
				sql_rows = self.__sql_cursor.fetchone()
			else:
				sql_rows = self.__sql_cursor.fetchmany(n_elements)
		else:
			raise Exception('Not connected to database')
		return sql_rows

	def InsertDataIntoSQLTable(self, table_name, data):
		'''
		 @brief Insert value to column in sql table.

		 @param table_name
		 @param data (Dictionary of column keys as dict keys with values, data = {'col_key': data})
		'''
		sql_command = "INSERT INTO " + table_name + " ("
		i = 0
		for key in data:
			sql_command += key
			if i < len(data)-1:
				sql_command += ", "
			i += 1
		sql_command += ") VALUES ("
		i = 0
		for key in data:
			sql_command += "%("+key+")s"
			if i < len(data)-1:
				sql_command += ", "
			i += 1
		sql_command += ")"
		self.ExecuteSQLCommand(sql_command, data)

	def AddColumnToSQLTable(self, table_name, column_value):
		'''
		 @brief Add column to existing table 

		 @param table_name
		 @param column_value (Defined as ("value_key", "value_type"))
		'''
		sql_command = "ALTER TABLE " + table_name + " ADD " + column_value[0] + " " + column_value[1]
		self.ExecuteSQLCommand(sql_command)

	def DropColumnFromSQLTable(self, table_name, column_key):
		'''
		 @brief Drop column from existing table 

		 @param table_name
		 @param column_key (Defined as column_key = "column_key")
		'''
		sql_command = "ALTER TABLE " + table_name + " DROP " + column_key
		self.ExecuteSQLCommand(sql_command)

	def RenameColumnFromSQLTable(self, table_name, column_key, new_column_value):
		'''
		 @brief Rename column in existing table 

		 @param table_name
		 @param column_key (Defined as column_key = "column_key")
		 @param new_column_value (Defined as ("value_key", "value_type"))
		'''
		sql_command = "ALTER TABLE " + table_name + " CHANGE " + column_key + " " + new_column_value[0] + " " + new_column_value[1]
		self.ExecuteSQLCommand(sql_command)

	def DropSQLTable(self, table_name):
		'''
		 @brief Drop table (delete entire table)

		 @param table_name
		'''
		sql_command = "DROP TABLE " + table_name
		self.ExecuteSQLCommand(sql_command)

	def RenameSQLTable(self, table_name, new_table_name):
		'''
		 @brief Rename table

		 @param table_name
		 @param new_table_name
		'''
		sql_command = "RENAME TABLE " + table_name + " TO " + new_table_name
		self.ExecuteSQLCommand(sql_command)

	def CreateSQLTable(self, table_name, columns, primary_key):
		'''
		 @brief Create new table

		 @param table_name
		 @param columns (Dictionary of column keys and value types, columns = {'col_key': 'value_type'}
		 @param primary_key (key in columns that is the primary key)
		'''
		sql_command = "CREATE TABLE IF NOT EXISTS " + table_name + " (" + primary_key + " " + columns[primary_key]
		for key in columns:
			if not(key == primary_key):
				sql_command += "," + key + " " + columns[key]
		sql_command += ", PRIMARY KEY("+ primary_key + "))"
		self.ExecuteSQLCommand(sql_command)

	def SelectSQLDatabase(self, database_name):
		'''
		 @brief Select database to use

		 @param database_name
		'''
		sql_command = "USE " + database_name
		self.ExecuteSQLCommand(sql_command)

	def CreateSQLDatabase(self, database_name):
		'''
		 @brief Create new database 

		 @param database_name
		'''
		sql_command = "CREATE DATABASE IF NOT EXISTS " + database_name
		self.ExecuteSQLCommand(sql_command)

	def DropSQLDatabase(self, database_name):
		'''
		 @brief Drop database (delete entire database)

		 @param database_name
		'''
		sql_command = "DROP DATABASE " + database_name
		self.ExecuteSQLCommand(sql_command)

	def FindKeyWord(self, sql_rows, key_word):
		'''
		 @brief Find key word in returned sql data 

		 @return True/False
		'''
		if isinstance(sql_rows, tuple):
			for sql_row in sql_rows:
				if self.FindKeyWord(sql_row, key_word):
					return True
		else:
			if isinstance(sql_rows, str):
				if key_word.upper() == sql_rows.upper():
					return True
		return False

	def CheckSQLDatabaseExist(self, database_name):
		'''
		 @brief Check if database exist

		 @param database_name

		 @return True/False
		'''
		sql_rows 	= self.GetSQLDatabases()
		exist 		= self.FindKeyWord(sql_rows, database_name)
		return exist

	def CheckSQLTableExist(self, table_name):
		'''
		 @brief Check if table exist

		 @param table_name

		 @return True/False
		'''
		sql_rows 	= self.GetSQLTables()
		exist 		= self.FindKeyWord(sql_rows, table_name)
		return exist

	def CheckSQLColumnExist(self, table_name, column_key):
		'''
		 @brief Check if column exist

		 @param table_name
		 @param column_key

		 @return True/False
		'''
		sql_rows 	= self.GetSQLColumns(table_name)
		exist 		= self.FindKeyWord(sql_rows, column_key)
		return exist

	def GetSQLDatabases(self):
		'''
		 @brief Get all sql databases

		 @return sql_rows
		'''
		sql_command = "SHOW DATABASES"
		self.ExecuteSQLCommand(sql_command)
		sql_rows = self.FetchSelectedSQLData()
		return sql_rows

	def GetSQLTables(self):
		'''
		 @brief Get all sql tables

		 @return sql_rows
		'''
		sql_command = "SHOW TABLES"
		self.ExecuteSQLCommand(sql_command)
		sql_rows = self.FetchSelectedSQLData()
		return sql_rows

	def GetSQLColumns(self, table_name):
		'''
		 @brief Get all sql columns from table

		 @param table_name

		 @return sql_rows
		'''
		sql_command = "SHOW COLUMNS FROM " + table_name
		self.ExecuteSQLCommand(sql_command)
		sql_rows = self.FetchSelectedSQLData()
		return sql_rows

	def ConvertSQLRowsToList(self, sql_rows, sql_list=[]):
		'''
		 @brief Convert sql rows to list

		 @param sql_rows

		 @return sql_list
		'''
		if isinstance(sql_rows, tuple):
			for sql_row in sql_rows:
				sql_list = self.ConvertSQLRowsToTuple(sql_row, sql_list)
		else:
			sql_list.append(sql_rows)
		return sql_list

	def ExecuteSQLCommand(self, sql_command, values=None):
		'''
		 @brief Execute sql_command to sql database

		 @param sql_command (sql string sql_command)
		 @param values (tuples of insert values. None if no values to insert (default=None))
		'''
		if not(self.GetSQLConnected()):
			self.OpenSQL()
		try:
			if isinstance(values, tuple) or isinstance(values, list) or isinstance(values, dict):
				self.__sql_cursor.execute(sql_command, values)
			else:
				self.__sql_cursor.execute(sql_command)
			self.__sql_con.commit()
			executed = True
		except mdb.Error, err:
			raise Exception("Error executing sql command to SQL database - \n\n"+sql_command+"\n\n - %d: %s" % (err.args[0],err.args[1]))

	def PrintSelectedSQLData(self):
		'''
		 @brief Print selected sql data 
		'''
		self.PrintFetchedSQLData(self.FetchSelectedSQLData())

	def PrintFetchedSQLData(self, sql_rows):
		'''
		 @brief Print fetched sql data

		 @param sql_rows (as returned from FetchSelectedSQLData())
		'''
		if isinstance(sql_rows, tuple):
			for sql_row in sql_rows:
				self.PrintFetchedSQLData(sql_row)
		else:
			print sql_rows

	def GetSQLConnected(self):
		'''
		 @brief Get True/False for connected (ready to use) sql database.

		 @return True/False
		'''
		return self.__sql_db_connected

	def SetSQLConfig(self, username, password, database=None, host="localhost"):
		'''
		 @brief Set config parameters for the sql database

		 @param username
 		 @param password
 		 @param database (None for not created yet, (default=None))
 		 @param host (default="localhost")
		'''
		self.__config = {
		  "username": username,
		  "password": password,
		  "database": database,
		  "host": host
		}

	def OpenSQL(self):
		'''
		 @brief Connect and initialize SQL database
		'''
		try:
			self.__sql_con 			= mdb.connect(host=self.__config["host"], user=self.__config["username"], passwd=self.__config["password"])
			self.__sql_cursor 		= self.__sql_con.cursor()
			self.__sql_db_connected = True
		except mdb.Error, err:
			raise Exception("Error connecting to SQL database - %d: %s" % (err.args[0],err.args[1]))
		if not(self.__config["database"] == None):
			if not(self.CheckSQLDatabaseExist(self.__config["database"])):
				self.CreateSQLDatabase(self.__config["database"])
			self.SelectSQLDatabase(self.__config["database"])

	def CloseSQL(self):
		'''
		 @brief Disconnect sql database
		'''
		if self.__sql_db_connected:
			self.__sql_cursor.close()
			self.__sql_con.close()
		self.__sql_db_connected = False

	def __del__(self):
		'''DESTRUCTOR'''
		self.CloseSQL()