'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''
import sys, warnings, unittest
from Test_main_ImportTestScripts.ImportDroneVisionScripts import ImportDroneVisionScripts
from Test_main_ImportTestScripts.ImportMasterSlaveScripts import ImportMasterSlaveScripts
from Test_main_ImportTestScripts.ImportDroneMasterSlaveScripts import ImportDroneMasterSlaveScripts
from Test_main_ImportTestScripts.ImportDataBaseScripts import ImportDataBaseScripts
from Test_main_ImportTestScripts.ImportBinScripts import ImportBinScripts

class Test_main():
	def __init__(self):
		'''CONSTRUCTOR'''
		self.SetAllKey()
		self.__run_test = False

		#warnings.filterwarnings('error') # Raise all warnings as exceptions

	def SetAllKey(self, all_key='all'):
		'''
		 @brief Set all key

		 @param all_key (default='all')
		'''
		self.__all_key = all_key

	def GetAllKey(self):
		'''
		 @brief Get all key
		'''
		return self.__all_key

	def CheckAllTests(self):
		'''
		 @brief Check if all tests are initiated

		 @return True/False
		'''
		all_tests = False
		if self.GetAllKey() in sys.argv:
			all_tests = True
		return all_tests

	def ImportTestScipts(self):
		'''
		 @brief Import test scripts
		 	- Continue adding new test scripts in the list
		'''
		self.__all_test_scripts = {'DroneVision': ImportDroneVisionScripts(), \
									'MasterSlave': ImportMasterSlaveScripts(), \
									'DroneMasterSlave': ImportDroneMasterSlaveScripts(), \
									'bin': ImportBinScripts(), \
									'DataBase': ImportDataBaseScripts()}

	def StartTest(self):
		'''
		 @brief Main start test function.
		 	Starts test as specified by user input.
		'''
		loader 	= unittest.TestLoader()
		suite 	= unittest.TestSuite()
		for test_scripts_key in self.__all_test_scripts:
			self.CheckTests(self.__all_test_scripts[test_scripts_key], suite, loader)

		if self.__run_test:
			unittest.TextTestRunner(verbosity=2).run(suite)
		else:
			for test_scripts_key in self.__all_test_scripts:
				self.PrintTestPossibilities(test_scripts_key)
			print '\nNo test input detected. Please append name of script to test, or "all" to test every script.'

	def CheckTests(self, test_scripts, suite, loader):
		'''
		 @brief Check if user specifies to test a unit from the given package.

		 @param test_scripts (dictionary)
		 @param suite (testtunits suite)
		 @param loader (testtunits loader)
		'''
		for key in test_scripts:
			if key in sys.argv or self.CheckAllTests():
				self.__run_test = True
				suite.addTest(loader.loadTestsFromTestCase(test_scripts[key]))

	def PrintTestPossibilities(self, test_scripts_key):
		'''
		 @brief Print possible test keys

		 @param test_scripts_key
		'''
		print 'Possible test units from ' + test_scripts_key + ':'
		for test_script_key in self.__all_test_scripts[test_scripts_key]:
			print test_script_key
		print '\n'


