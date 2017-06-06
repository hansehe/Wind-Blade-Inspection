'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

from AutoPip.AutoPip import AutoPip

'''
	Continue the list with necessary packages which are required.
'''
requirement_list = ['cython', 
					'pyserial',
					'numpy',
					'matplotlib',
					'scipy',
					'filterpy',
					'sklearn',
					'jsonpickle',
					'pandas',
					'MySQL-python']

'''
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
'''

'''
 @brief Execute to check and install requirements if necessary, as listed above.
'''
def install_requirements():
	global requirement_list
	autoPip_obj = AutoPip()
	autoPip_obj.ImportPackages(requirement_list)