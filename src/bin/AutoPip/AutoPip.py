'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

import sys, time, os, site, importlib
from . get_pip import get_pip_main

'''
 @brieaf This class implements auto import of packages installed using pip
  Note! Run with 'sudo' when executing this class due to linux restrictions.
'''
class AutoPip():
	def __init__(self):
		'''CONSTRUCTOR'''
		self.__flat_installed_packages = []
		self.ImportPip()
		self.SetInstalledPackages()

	def ImportPip(self):
		'''
		 @brief Import pip. 
		  Will install pip if necessary, and restarts the program after installment.
		'''
		try:
			globals()['pip'] = importlib.import_module('pip')
		except ImportError:
			restart_cmd = sys.argv
			sys.argv = [sys.argv[0]]
			get_pip_main()
			# Restart program.
			os.execv(sys.executable, ['python'] + restart_cmd)
			sys.exit()

	def SetInstalledPackages(self):
		'''
		 @brief Set installed pip packages.
		  Check installed pip packages by using self.CheckPackage().
		'''
		installed_packages = pip.get_installed_distributions()
		self.__flat_installed_packages = [package.project_name for package in installed_packages]

	def CheckPackage(self, package):
		'''
		 @brief Check if package is installed

		 @param Name of package (str)
		 @return True/False
		'''
		ok = False
		if package in self.__flat_installed_packages:
			ok = True
		return ok

	def ImportPackage(self, package):
		'''
		 @brief install package using pip
		'''
		if not(self.CheckPackage(package)):
			print 'installing '+package
			pip.main(['install', package])

	def ImportPackages(self, packages):
		'''
		 @brief Install and import multiple packages

		 @param packages Array of package names (str)
		'''
		for package in packages:
			self.ImportPackage(package)
