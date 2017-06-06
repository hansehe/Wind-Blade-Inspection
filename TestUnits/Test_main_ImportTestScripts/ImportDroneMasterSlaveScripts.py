'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

def ImportDroneMasterSlaveScripts():
	'''
	 @brief Import test modules from DroneMasterSlave scripts.
	'''
	from TestUnits.Test_src.Test_DroneMaster import Test_DroneMaster
	from TestUnits.Test_src.Test_DroneSlave import Test_DroneSlave

	DroneMasterSlaveSripts = {
		'DroneMaster': Test_DroneMaster,
		'DroneSlave': Test_DroneSlave
	}

	return DroneMasterSlaveSripts