'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''
import os, threading, timeit, time
from datetime import datetime

def RemoveDir(directory):
	'''
	 @brief Delete everything reachable from the directory named in 'top',
			assuming there are no symbolic links.
			CAUTION:  This is dangerous!  For example, if top == '/', it
			could delete all your disk files.
	 @param directory
	'''
	if os.path.isdir(directory):
		for root, dirs, files in os.walk(directory, topdown=False):
			for name in files:
				os.remove(os.path.join(root, name))
			for name in dirs:
				os.rmdir(os.path.join(root, name))

def CheckDir(directory):
	'''
	 @brief Create directory

	 @param directory Name of directory.
	'''
	if not(os.path.isdir(directory)):
		try:
			os.makedirs(directory)
		except Exception, err:
			warnings.simplefilter('always')
			warnings.warn(str(err), Warning)
			warnings.simplefilter('default')

def GetTimestampedFolder(timestamp, output_folder, sub_output_folder):
	'''
	 @brief Get timestamp folder for this session.

	 @param timestamp String
	 @param output_folder
	 @param sub_output_folder

	 @return output_folder (updated)
	'''
	if len(sub_output_folder) > 0:
		output_folder += sub_output_folder
	output_folder += timestamp + '/'
	return output_folder

def GetTimestamp(fractional=False):
	'''
	 @brief Get string timestamp

	 @param fractional (return timestamp with fractional seconds (default=False))

	 @return string timestamp 
	'''
	str_time = "%Y_%m_%d__%H_%M_%S"
	if fractional:
		timestamp = datetime.utcnow().strftime(str_time + ".%f")
	else:
		timestamp = datetime.utcnow().strftime(str_time)
	return timestamp

def RunThread(func, args=(), daemon=True, lock=None, wait_lock=False, wait_timeout=0.5):
	'''
	 @brief Run function in separate thread.

	 @param func Function to run.
	 @param args Input arguments (single argument: (arg1,) - or multiple: (arg1, arg2))
	 @param daemon (True/false) - run as daemon or not.
	 @param lock Mutex lock - wait for it to be available before starting thread.
	 @param wait_lock - Wait for mutex to be available, or raise exception immideately.
	 @param wait_timeout - Time to wait in seconds (if wait_lock==True)

	 return t Threading object
	'''
	t = threading.Thread(target=func, args=args)
	t.daemon = daemon
	if lock == None:
		t.start()
	else:
		if wait_lock:
			timeout = timeit.default_timer()
			while True:
				if not(lock.locked()):
					t.start()
					break
				elif (timeit.default_timer() - timeout) > wait_timeout:
					raise Exception('Runtime error - lock not availble')
		else:
			if not(lock.locked()):
				t.start()
			else:
				raise Exception('Runtime error - lock not availble')
	return t