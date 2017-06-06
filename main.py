#!/usr/bin/python
'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)

 Usage:
 	'sudo python main.py' or 'sudo ./main.py'
 	Append either of these options:
 		- 'run' to initiate main program, and append one of these options:
 			- 'master' if this module is master.
 			- 'slave' if this module is slave.
 			- 'simulate' to simulate a slave + master program flow on a single module.
 				-> Additionally append either of these commands to the master device to initiate as follows:
 					- 'calibrate_stereopsis' to start a new session aimed at capturing frames of a chessboard for calibration purposes
 					- 'calibrate_blob' to start a new session aimed at capturing frames of structured light and normal frames to calibrate blob sizes and distance.
 					- 'calibrate' to start a new session aimed at both of the alternatives above.
 		- 'test' to start a unit test. Specify with name of script to test the unit, or 'all' to test all units.

 	Append 'install' if required packages needs to be installed (does not include opencv, openGL or mysql - see HOWTO.txt).
 	Linux distributions usually needs 'sudo' command in front for permission privileges.
	
	MySQL:
		- Start mysql server (must be started before running this script): sudo service mysql start 
 		- Stop mysql server: sudo service mysql stop
 		- Status mysql server: sudo service mysql status 
 		- manual login to mysql: mysql -u root -p
 			- Where root is username, and password where set when mysql was installed
'''

import sys, threading, warnings

'''
 Install requirements before importing libraries.
'''
if __name__ == "__main__":
	if 'install' in sys.argv:
		from src.bin.requirements import install_requirements
		install_requirements()

from src.bin.tools import RunThread
from src.DroneMaster import DroneMaster
from src.DroneSlave import DroneSlave
from Settings.Settings import Settings

def RunMaster(calibrate_stereopsis_session=False, calibrate_blob_scale_detector_session=False, preset_settings=None):
	'''
	 @brief Shortcut for running master

 	 @param calibrate_stereopsis_session (Set calibrate_stereopsis_session to True for starting a new stereopsis calibration session (default=False))
 	 @param calibrate_blob_scale_detector_session (Set calibrate_blob_scale_detector_session to True for starting a new blob scale detector calibration session (default=False))
	 @param preset_settings (Settings class for giving preset settings (default=None). None means no preset settings)
	'''
	droneMaster 	= DroneMaster(settings_inst=preset_settings, calibrate_stereopsis_session=calibrate_stereopsis_session, calibrate_blob_scale_detector_session=calibrate_blob_scale_detector_session)
	error 			= None
	try:
		droneMaster.InitializeMaster()
		droneMaster.RunMaster()
	except Exception, err:
		error = err
	del droneMaster
	if not(isinstance(error, type(None))):
		raise

def RunSlave(preset_settings=None):
	'''
	 @brief Shortcut for running slave

	 @param preset_settings (Settings class for giving preset settings (default=None). None means no preset settings)
	'''
	droneSlave 		= DroneSlave(settings_inst=preset_settings)
	error 			= None
	try:
		droneSlave.InitializeSlave()
		droneSlave.RunSlave()
	except Exception, err:
		error = err
	del droneSlave
	if not(isinstance(error, type(None))):
		raise

def PrintInfo():
		'''
		 @brief Print info about how to use this program.
		'''
		preset_settings = Settings(skip_checking_for_user_input_cmd_line=True, skip_checking_for_user_input=True)
		static_settings = preset_settings.GetStaticSettings()
		print "#------------------------ INFO ------------------------#"
		print "# Start program by typing 'sudo python main.py' or './main.py', and append following options:"
		print "# \t Append 'run' with either 'master' or 'slave' to start the main program."
		print "# \t\t Starting master device: 'sudo python main.py run master'."
		print "# \t\t Starting slave device: 'sudo python main.py run slave'."
		print "# \t Calibration of the stereopsis system and/or the standard distance between blobs may be initiated by appending following after 'run master':"
		print "# \t\t Calibrating both the stereopsis system and the standard distance between blobs, append: 'calibrate'"
		print "# \t\t Calibrating stereopsis system only, append: 'calibrate_stereopsis'"
		print "# \t\t Calibrating standard distance between blobs only, append: 'calibrate_blobs'"
		print "# \t\t (Hint: a full calibration reset may be initiated by just entering one of the calibration commands, and stepping through without capturing any new calibration frames.)"
		print "# \t Change a specifig setting (may be a mix of lower or upper case):"
		print "# \t\t -SETTING_TYPE --SETTING ---NEW_SETTING_VALUE"
		print "# \t Change a specific setting by entering a valid python code, may be commenced by typing: "
		print "# \t\t -SETTING_TYPE --SETTING"
		print "# Saved sessions may be simulated by typing 'simulate video' or 'simulate image' instead of 'master'/'slave', with video or image targets given by the settings configurations."
		print "# Program settings may be changed in detail by changing the settings configuration json file."
		print "# Program settings may be found at {0}.json.".format(static_settings['settings_params_folder']+static_settings['settings_params_fname'])
		print "# Append 'configure_settings' to configure settings in the program before program start (must be consistently done on both the master device and slave device)."
		print "# Append 'reset_settings' to reset settings to default, as specified by the SettingsConfigured unit (must be done on both the master device and slave device)."
		print "#------------------------------------------------------#"
		answer = raw_input("# Would you like to see the settings information? [Y/n]: ")
		if answer.upper() == 'Y':
			preset_settings.PrintSettingsInfo(ask_user_for_settings_parameters=True)

if __name__ == "__main__":
	'''START PROGRAM'''
	if 'info' in sys.argv:
		PrintInfo()
	if 'reset_settings' in sys.argv:
		Settings(reset_settings=True, skip_checking_for_user_input_cmd_line=True, skip_checking_for_user_input=True)
		print '# Settings were reset to default values...'
	if 'configure_settings' in sys.argv:
		Settings(skip_checking_for_user_input_cmd_line=True, skip_checking_for_user_input=True).PrintSettingsInfo(configure_settings=True, ask_user_for_settings_parameters=True)

	if 'run' in sys.argv:
		calib_stereopsis = False
		calib_blobs 		 = False
		if 'calibrate' in sys.argv:
			calib_stereopsis = True
			calib_blobs 		 = True
		if 'calibrate_stereopsis' in sys.argv:
			calib_stereopsis = True
		if 'calibrate_blobs' in sys.argv:
			calib_blobs 	 = True

		if 'master' in sys.argv:
			RunMaster(calib_stereopsis, calib_blobs)
		elif 'slave' in sys.argv:
			RunSlave()
		elif 'simulate' in sys.argv:
			preset_settings = Settings()
			if 'video' in sys.argv:
				preset_settings.ChangeSetting('BASIC', 'source_type', 'VIDEO')
			elif 'image' in sys.argv:
				preset_settings.ChangeSetting('BASIC', 'source_type', 'IMAGE')
			else:
				raise Exception("Please append 'video' or 'image' to select a source type for simulation, and be sure to set correct video/image targets in the settings configuration file. Append 'info' to get more details.")
			preset_settings.ChangeSetting('TCP', 'master_ip', 'localhost')
			if not(preset_settings.GetSettings('REAL_TIME_PLOT', 'use_matplotlib')):
				warnings.warn('PyQtImage does not work outside of main thread - switching to matplotlib with interactive mode on.')
				preset_settings.ChangeSetting('REAL_TIME_PLOT', 'use_matplotlib', True)
				preset_settings.ChangeSetting('REAL_TIME_PLOT', 'use_interactive_mode', True)
			RunThread(RunSlave, args=(preset_settings,))
			RunMaster(calib_stereopsis, calib_blobs, preset_settings)
		else:
			raise Exception("Please append 'master', 'slave' or 'simulate' to start the main program.")
	elif 'test' in sys.argv:
		from TestUnits.Test_main import Test_main
		test_main = Test_main()
		test_main.ImportTestScipts()
		test_main.StartTest()
	else:
		Settings(skip_checking_for_user_input=True)
		warnings.warn("No start command given, please append 'info' to get more detailed information on how to use this program.")