'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

#Import configured settings
from SettingsConfigured import SettingsConfigured
from src.bin.tools import CheckDir

from getpass import getpass
import sys, json, os, warnings

'''
 @brief Settings class.

 @param reset_settings (True/False - reset settings to default values (default=None))
 @param load_initial_settings (default=True)
 @param skip_checking_for_user_input_cmd_line (True/False - default=False)
 @param skip_checking_for_user_input (True/False - default=False)
'''
class Settings(SettingsConfigured):
	def __init__(self, reset_settings=None, load_initial_settings=True, skip_checking_for_user_input_cmd_line=False, skip_checking_for_user_input=False):
		'''CONSTRUCTOR'''
		SettingsConfigured.__init__(self, load_initial_settings=load_initial_settings)
		if load_initial_settings:
			self.GetInitialSettings(reset_settings=reset_settings, skip_checking_for_user_input_cmd_line=skip_checking_for_user_input_cmd_line, skip_checking_for_user_input=skip_checking_for_user_input)

	def GetStaticSettings(self):
		'''
		 @brief Get static settings
		'''
		return dict(self._static_settings)

	def GetInitialSettings(self, reset_settings=None, settings_filename=None, skip_checking_for_user_input_cmd_line=False, skip_checking_for_user_input=False):
		'''
		 @brief Get initial settings from given json file, or from default values

		 @param reset_settings (True/False (default=None))
		 @param settings_filename (default=None)
		 @param skip_checking_for_user_input_cmd_line (True/False - default=False)
		 @param skip_checking_for_user_input (True/False - default=False)
		'''
		if not(isinstance(reset_settings, bool)):
			reset_settings = self._static_settings['reset_settings']

		settings_changed = False
		if reset_settings or not(self.LoadSettingsFromJson(settings_filename)):
			self.ResetSettings()
			settings_changed = True
		if not(skip_checking_for_user_input_cmd_line):
			if self.CheckForUserInputSettingsFromTerminal():
				settings_changed = True
		if settings_changed:
			self.SaveSettingsToJson(settings_filename)
		if not(skip_checking_for_user_input):
			self.CheckForUserInputSettings()

	def GetDefaultSettingsFilename(self):
		'''
		 @brief Get default save settings filename

		 @return filename
		'''
		CheckDir(self._static_settings['settings_params_folder'])
		filename = self._static_settings['settings_params_folder'] + self._static_settings['settings_params_fname']
		return filename

	def SaveSettingsToJson(self, filename=None):
		'''
		 @brief Save settings to json file

		 @param filename (Without .json file ending)
		'''
		if not(isinstance(filename, str)):
			filename = self.GetDefaultSettingsFilename()
		filename += '.json'
		with open(filename, 'w') as f:
			try:
				json.dump(self._settings, f, indent=4, sort_keys=True)
			except:
				print 'JSON dump failed: ', self._settings
				raise

	def LoadSettingsFromJson(self, filename=None):
		'''
		 @brief Load settings from json file

		 @param filename (Without .json file ending)

		 @return True/False (loaded successfully)
		'''
		if not(isinstance(filename, str)):
			filename = self.GetDefaultSettingsFilename()
		success = False
		filename += '.json'
		if os.path.isfile(filename):
			self.ResetSettings()
			old_settings = dict(self._settings) # Need to convert json converted value types to correct value types
			with open(filename, 'r') as f:
				try:
					self._settings = json.load(f)
					success = True
				except Exception, err:
					warnings.simplefilter('always')
					warnings.warn(str(err), Warning)
					warnings.simplefilter('default')
			if success:
				for main_key in self._settings:
					for sub_key in self._settings[main_key]:
						if isinstance(old_settings[main_key][sub_key], type(None)) and isinstance(self._settings[main_key][sub_key], unicode): # Some string settings may be set to None
							self._settings[main_key][sub_key] = str(self._settings[main_key][sub_key])
						elif not(isinstance(old_settings[main_key][sub_key], type(None))) and not(isinstance(self._settings[main_key][sub_key], type(None))):
							self._settings[main_key][sub_key] = type(old_settings[main_key][sub_key])(self._settings[main_key][sub_key])
		return success

	def ResetSettings(self, new_settings=None):
		'''
		 @brief Reset the settings. Local initial settings are used if no new settings are given.

		 @param new_settings (Dictionary (default=None))
		'''
		if isinstance(new_settings, dict):
			self._settings = new_settings
		else:
			self.SetInitialSettings()

	def ChangeSetting(self, header, setting, value):
		'''
		 @brief Safely change a setting.
		 	- header 	= Main setting header (f.ex 'BASIC')
		 	- setting 	= setting type (f.ex 'reset_calibration' in 'BASIC')
		 	- value 	= new setting value (Be sure to give correct value type, since this is not assterted.)
		 	Raises ValueError if header or setting is not present in the settings dictionary.
		 @param header
		 @param setting
		 @param value
		'''
		if setting == None:
			raise ValueError('Setting {0} is not a part of the settings[{1}]!'.format(setting, header))
		self.AssertValidSetting(header, setting)
		if not(isinstance(value, type(self._settings[header][setting]))):
			raise ValueError('New setting value is not of the same type as the old one: Old type = {0}, new type = {1}, from setting[{2}][{3}]'.format(type(self._settings[header][setting]), type(value), header, setting))
		self._settings[header][setting] = value

	def GetRawSettings(self):
		'''
		 @brief Get raw settings dictionary

		 @return settings
		'''
		return self._settings

	def GetSettings(self, header=None, setting=None):
		'''
		 @brief Safely get settings.
		 	- header 	= Main setting header (f.ex 'BASIC') - If None, then new settings instance of all settings are returned (default=None).
		 	- setting 	= setting type (f.ex 'reset_calibration' in 'BASIC') - If None, then new settings instance of main setting header is returned (default=None).
		 	Raises ValueError if header or setting is not present in the settings dictionary.
		 @param header
		 @param setting

		 @return settings_val
		'''
		if header == None:
			settings_val = Settings(load_initial_settings=False)
			settings_val.ResetSettings(self._settings)
		else:
			self.AssertValidSetting(header, setting)
			if setting == None:
				if isinstance(self._settings[header], dict):
					settings_val = Settings(load_initial_settings=False)
					settings_val.ResetSettings(self._settings[header])
				else:
					settings_val = self._settings[header]
			else:
				settings_val = self._settings[header][setting]
		return settings_val

	def AssertValidSetting(self, header, setting=None):
		'''
		 @brief Assert valid setting request.
		 	- header 	= Main setting header (f.ex 'BASIC')
		 	- setting 	= setting type (f.ex 'reset_calibration' in 'BASIC') - If None, then it is not checked.
		 	Raises ValueError if header or setting is not present in the settings dictionary.

		  @param header
		  @param setting
		  @param value
		'''
		if not(header in self._settings):
			raise ValueError('Header {0} is not a part of the settings!'.format(header))
		if not(setting == None):
			if not(setting in self._settings[header]):
				raise ValueError('Setting {0} is not a part of the settings[{1}]!'.format(setting, header))

	def CheckForUserInputSettings(self):
		'''
		 @brief Check for settings user input at bootup
		'''
		checkdatabaseKeys 	= ['username', 'password', 'database', 'output_folder', 'sub_output_folder']
		database_key 		= 'DATABASE'
		for key in checkdatabaseKeys:
			if self._settings[database_key][key] == None:
				self._settings[database_key][key] = self.UserSettingsInput(database_key, key)

	def UserSettingsInput(self, settings_key, key):
		'''
		 @brief Get user input to set key in settings

		 @param key

		 @return setting
		'''
		user_text = 'Enter ' + settings_key + ' ' + key + ': '
		if key == 'password':
			setting = getpass(user_text)
		else:
			setting = raw_input(user_text)
		return setting

	def CheckForUserInputSettingsFromTerminal(self):
		'''
		 @brief Check for user input settings directly from command line.
		 	User may change a setting by adding following in the terminal command:
		 		-SETTING_TYPE 			(f.ex -BASIC)
		 		--SETTING 				(f.ex --source_type)
		 		---NEW_SETTING_VALUE	(f.ex ---CAMERA)
		 	Example: -BASIC --source_type ---CAMERA
		 	Note: empty NEW_SETTING_VALUE results on asking user for a python value to set, f.ex: -BASIC --source_type ---
		 	Note: the inputs are forcely uppercased for header and lowercased for setting. For comparison, so '-basic' == 'BASIC' and '--SOURCE_TYPE' == 'source_type'.
		
		 @return True/False (True if settings changed, False if not)
		'''
		new_setting_cmd 	= '-'
		settings_changed 	= False
		i = 0
		while i < len(sys.argv):
			if sys.argv[i].count(new_setting_cmd) == 1 and (i+1 < len(sys.argv)):
				header = sys.argv[i][1:].upper()
				i += 1
				if sys.argv[i].count(new_setting_cmd) == 2:
					setting 	= sys.argv[i][2:].lower()
					old_value 	= self.GetSettings(header, setting)
					if type(old_value) == dict or type(old_value) == list or type(old_value) == tuple:
						print "# Old settings type was of {0}, and cannot be set from the command line.".format(type(old_value))
						if self.AskUserToConfigureSetting(header, setting):
							settings_changed = True
					elif i+1 < len(sys.argv):
						i += 1
						if sys.argv[i].count(new_setting_cmd) == 3:
							new_value = sys.argv[i][3:]
							if type(old_value) == bool:
								if new_value.lower() == 'true' or new_value.lower() == '1':
									new_value = True
								elif new_value.lower() == 'false' or new_value.lower() == '0':
									new_value = False
								else:
									warnings.simplefilter('always')
									warnings.warn("Bool setting from [{0}][{1}] are given as: '---false' || '---0' <-> '---true' || '---1' (upper/lower or mixed cased). You typed this value: {2}".format(header, setting, new_value), Warning)
									warnings.simplefilter('default')
									if self.AskUserToConfigureSetting(header, setting):
										settings_changed = True
							else:
								new_value = type(old_value)(new_value)
							try:
								self.ChangeSetting(header, setting, new_value)
								settings_changed = True
								print '#------------- Changed setting[{0}][{1}] from {2} -> {3} -------------#'.format(header, setting, old_value, new_value)
							except Exception, err:
								warnings.simplefilter('always')
								warnings.warn(str(err), Warning)
								warnings.simplefilter('default')
								if self.AskUserToConfigureSetting(header, setting):
									settings_changed = True
						else:
							if self.AskUserToConfigureSetting(header, setting):
								settings_changed = True
							i -= 1
					else:
						if self.AskUserToConfigureSetting(header, setting):
							settings_changed = True
			i += 1
		return settings_changed

	def PrintSettingsInfo(self, configure_settings=False, ask_user_for_settings_parameters=False):
		'''
		 @brief Print settings information and configure settings if specified.

		 @param configure_settings (Default=False)
		 @param ask_user_for_settings_parameters (default=False)
		'''
		settings_info 		= self.GetSettingsInfo()
		settings_changed 	= False
		if configure_settings:
			print "#------------------------------------------ CONFIGURE SETTINGS ------------------------------------------#"
		else:
			print "#----------------------------------------- SETTINGS INFORMATION -----------------------------------------#"
		for main_settings_key in settings_info:
			print "# Settings type: {0}".format(main_settings_key)
			for sub_settings_key in settings_info[main_settings_key]:
				print "# \t - Setting: {0} \t\t\t - {1}".format(sub_settings_key, settings_info[main_settings_key][sub_settings_key])
				if configure_settings:
					if self.AskUserToConfigureSetting(main_settings_key, sub_settings_key, n_tabs=2):
						settings_changed = True
		print "#------------------------------------- STATIC SETTINGS INFORMATION --------------------------------------#"
		static_settings_info = self.GetStaticSettingsInfo()
		print "# Following settings are static and may only be configured in the SetttingsConfigured unit."
		for static_settings_key in static_settings_info:
			print "# \t - Static setting: {0} \t\t\t - {1}".format(static_settings_key, static_settings_info[static_settings_key])
		print "#--------------------------------------------------------------------------------------------------------#"
		if configure_settings and settings_changed:
			CheckDir(self._static_settings['settings_params_folder'])
			settings_filename = self._static_settings['settings_params_folder'] + self._static_settings['settings_params_fname']
			self.SaveSettingsToJson(settings_filename)
		if ask_user_for_settings_parameters:
			answer = raw_input("# Would you like to see the settings parameters? [Y/n]: ")
			if answer.upper() == 'Y':
				self.PrintSettings()

	def PrintSettings(self):
		'''
		 @brief Print settings
		'''
		print "#----------------------------------------- SETTINGS PARAMETERS ------------------------------------------#"
		for main_settings_key in self._settings:
			print "# Settings type: {0}".format(main_settings_key)
			for sub_settings_key in self._settings[main_settings_key]:
				print "# \t - Setting: {0} \t\t\t - {1}".format(sub_settings_key, self._settings[main_settings_key][sub_settings_key])
		print "#--------------------------------------------------------------------------------------------------------#"

	def AskUserToConfigureSetting(self, main_settings_key, sub_settings_key, n_tabs=0, skip_asking_user_to_configure=False):
		'''
		 @brief Ask user to configure a specific setting.

		 @param main_settings_key
		 @param sub_settings_key
		 @param n_tabs (default=0)
		 @param skip_asking_user_to_configure (True/False - default=False)

		 @return True/False (True if settings changed, False if not)
		'''
		settings_changed 	= False
		tabs 				= '\t'*n_tabs
		answer 				= 'Y'
		if not(skip_asking_user_to_configure):
			answer = raw_input("#{0} Would you like to configure setting[{1}][{2}]? [Y/n]: ".format(tabs, main_settings_key, sub_settings_key))
		if answer.upper() == 'Y':
			settings_type 	= type(self._settings[main_settings_key][sub_settings_key])
			fail_msg 		= "Please enter a valid python value of type {0}".format(settings_type)
			configured 		= False
			while not(configured):
				try:
					new_setting = input("#{0} Please enter new setting of {1} as python value: ".format(tabs, settings_type))
				except:
					print "#{0} InputError: ".format(tabs) + fail_msg
					continue
				if settings_type != type(new_setting) and not(settings_type == type(None) and isinstance(new_setting, str)) and not(settings_type == str and isinstance(new_setting, type(None))):
					print "#{0} ValueError: ".format(tabs) + fail_msg
				else:
					old_setting = self._settings[main_settings_key][sub_settings_key]
					self._settings[main_settings_key][sub_settings_key] = new_setting
					print "#{0} Changed setting[{1}][{2}] from {3} -> {4}".format(tabs, main_settings_key, sub_settings_key, old_setting, new_setting)
					configured = True
					settings_changed = True
		return settings_changed


