'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

def ImportMasterSlaveScripts():
	'''
	 @brief Import test modules from MasterSlave scripts.
	'''
	from TestUnits.Test_src.Test_MasterSlave.Test_Master import Test_Master
	from TestUnits.Test_src.Test_MasterSlave.Test_Slave import Test_Slave

	MasterSlaveSripts = {
		'Master': Test_Master,
		'Slave': Test_Slave
	}

	return MasterSlaveSripts