'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import os, pickle, fnmatch, json, copy
from tools import CheckDir
import numpy as np

'''
 @brief SaveParameters class. Generally used by subclasses to save and load parameters.

 @param save_folder
 @param save_fname (without .pickle at end)
'''
class SaveParameters():
	def __init__(self, save_folder, save_fname, save_to_json):
		'''CONSTRUCTOR'''
		self.__save_folder 	= save_folder
		self.__save_fname 	= save_fname
		self.__save_to_json = save_to_json
		CheckDir(self.__save_folder)

	def Save(self, dictionary):
		'''
		 @brief General save function

		 @param dictionary
		'''
		self.SaveToPickle(dictionary)
		if self.__save_to_json:
			self.SaveToJson(dictionary)

	def Load(self):
		'''
		 @brief General load function

		 @return ok, dictionary (True/False - calibration parameters loaded successfully, dictionary)
		'''
		ok, dictionary = self.LoadFromPickle()
		return ok, dictionary

	def MatchCalibFrames(self, origin_fname, match_frame_list, origin_fname_folder, match_frame_list_folder):
		'''
		 @brief Find match of origin_fname in match_frame_list

		 @param origin_fname
		 @param match_frame_list
		 @param origin_fname_folder
		 @param match_frame_list_folder

		 @return, match_fname
		'''
		match = False
		for match_fname in match_frame_list:
			if fnmatch.fnmatch(origin_fname[len(origin_fname_folder):], match_fname[len(match_frame_list_folder):]):
				match = True
				break
		if not(match):
			raise Exception('No matching frame in frames list - frame: {0}, frames {1}'.format(origin_fname, match_frame_list))
		return match_fname

	def SaveToPickle(self, dictionary):
		'''
		 @brief Save dictionary to pickle file

		 @param dictionary
		'''
		parameters = self.DumpParams(dictionary)
		if not(os.path.isdir('./'+self.__save_folder)):
			os.makedirs('./'+self.__save_folder)
		fname = self.__save_folder + self.__save_fname + '.pickle'
		with open(fname, 'wb') as f:
			pickle.dump(parameters, f)

	def LoadFromPickle(self):
		'''
		 @brief Load parameters from pickle file

		 @return dictionary
		'''
		parameters 	= []
		fname = self.__save_folder + self.__save_fname + '.pickle'
		if os.path.isfile(fname):
			with open(fname, 'rb') as f:
				parameters = pickle.load(f)
		dictionary = self.LoadParams(parameters)
		return dictionary

	def SaveToJson(self, dictionary):
		'''
		 @brief Save parameters to json readable file.

		 @param dictionary
		'''
		dict_parameters = self.DumpParams(dictionary, save_to_json=True)
		fname = self.__save_folder + self.__save_fname + '.json'
		with open(fname, 'w') as f:
			try:
				json.dump(dict_parameters, f, indent=4, sort_keys=True)
			except:
				print 'JSON dump failed: ', dict_parameters
				raise

	def LoadFromJson(self):
		'''
		 @brief Load parameters from json readable file.

		 @return dictionary
		'''
		dict_parameters = {}
		fname = self.__save_folder + self.__save_fname + '.json'
		if os.path.isfile(fname):
			with open(fname, 'r') as f:
				dict_parameters = json.load(f)
		dictionary = self.LoadParams(dict_parameters)
		return dictionary

	def CheckNumpyArray(self, variable):
		'''
		 @brief Check if variable is numpy array. Json cannot dump numpy arrays directly.

		 @return json_usable_variable
		'''
		if isinstance(variable, np.ndarray):
			return ['numpy_array', variable.tolist()]
		elif isinstance(variable, list) or isinstance(variable, tuple):
			if isinstance(variable, tuple):
				variable = list(variable)
			for i in range(len(variable)):
				variable[i] = self.CheckNumpyArray(variable[i])
		return variable

	def ListToNumpyArray(self, variable):
		'''
		 @brief Check if variable from json is list, and convert it to numpy array if necessary.

		 @param variable

		 @return variable (numpy array if it was a list)
		'''
		if isinstance(variable, list):
				if len(variable) == 2:
					if variable[0] == 'numpy_array':
						return np.array(variable[1])
				else:
					for i in range(len(variable)):
						variable[i] = self.ListToNumpyArray(variable[i])
		return variable

	def DumpParams(self, dictionary, save_to_json=False):
		'''
		 @brief Dump calibration parameters to dictionary or list
		 
		 @param dictionary, save_to_json

		 @return parameters
		'''
		cp_dict = copy.deepcopy(dictionary)
		if save_to_json:
			parameters = {}
			for key in cp_dict:
				parameters[key] = self.CheckNumpyArray(cp_dict[key])
		else:
			parameters = cp_dict
		return parameters

	def LoadParams(self, parameters, load_from_json=False):
		'''
		 @brief Load calibration parameters from dictionary or list

		 @param parameters, load_from_json

		 @param ok, dictionary (True/False for successfull load, dictionary with parameters)
		'''
		ok = False
		dictionary = {}
		if len(parameters) > 0:
			ok = True
			if load_from_json:
				for key in parameters:
					dictionary[key] = self.ListToNumpyArray(parameters[key])
			else:
				dictionary = parameters
		return ok, dictionary