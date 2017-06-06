'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

def ImportDataBaseScripts():
	'''
	 @brief Import test modules from DataBase scripts.
	'''
	from TestUnits.Test_src.Test_DataBase.Test_DataBase import Test_DataBase
	from TestUnits.Test_src.Test_DataBase.Test_MySQL.Test_MySQL import Test_MySQL
	from TestUnits.Test_src.Test_DataBase.Test_GPS.Test_GPS import Test_GPS
	from TestUnits.Test_src.Test_DataBase.Test_FrameRecorder.Test_FrameRecorder import Test_FrameRecorder

	DataBaseScripts = {
		'DataBase': Test_DataBase,
		'MySQL': Test_MySQL,
		'GPS': Test_GPS,
		'FrameRecorder': Test_FrameRecorder
	}

	return DataBaseScripts