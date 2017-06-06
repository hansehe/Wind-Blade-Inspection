'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

def ImportBinScripts():
	'''
	 @brief Import test modules from bin scripts.

	 @return BinScripts
	'''
	from TestUnits.Test_src.Test_bin.Test_UserInput.Test_UserInput import Test_UserInput

	BinScripts = {
		'UserInput': Test_UserInput
	}
	
	return BinScripts