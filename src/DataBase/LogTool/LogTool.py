'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''
import timeit
from getpass import getpass
from src.DroneVision.DroneVision_src.hardware.imageTools import MatplotShow

'''
 @brief LogTool class for logging and printing information.

 @param subclass DroneMaster or DroneSlave class (self)
 @param filename Log filename (default='')
 @param printToScreen True/False (default=False)
'''
class LogTool():
	def __init__(self, subclass, filename='', printToScreen=False):
		'''CONSTRUCTUR'''
		self.__subclass		 = subclass
		self.__filename 	 = filename
		self.__printToScreen = printToScreen
		self.ResetProcessData()

	def ResetProcessData(self):
		'''
		 @brief Reset process data by deleting all values
		'''
		self.__process_data = {}

	def SetProcessData(self, tag, value):
		'''
		 @brief Set data dictionary for storing data in the database 

		 @param tag
		 @param value
		'''
		self.__process_data[tag] = value

	def GetProcessData(self):
		'''
		 @brief Get process data dictionary
		
		 @return process_data
		'''
		return self.__process_data

	def SetLogFilename(self, filename):
		'''
		 @brief Set filename and open log for writing
		'''
		self.__filename = filename
		self.OpenLog()

	def OpenLog(self):
		'''
		 @brief Open log
		'''
		self._f = open(self.__filename, 'w')

	def Print(self, logText):
		'''
		 @brief Write log text, and print to screen if necessary.
		 	Appends new line automatically.

		 @param Log text
		'''
		self._f.write(logText + '\n')
		if self.__printToScreen:
			print logText

	def CloseLog(self):
		'''
		 @brief Close log
		'''
		self._f.close()

	def PrintData(self, process_data, points3D=[]):
		'''
		 @brief Print process data 

		 @param process_data (dictionary, {'tag': data}, None if nothing to print)
		 @param points3D (for printing 3D points (default=[]))
		'''
		if not(isinstance(process_data, dict)):
			process_data = self.GetProcessData() # Use local process data
		if len(process_data) > 0:
			self.Print('# Process data:')
			for key in process_data:
				self.Print('# \t - '+key+': \t {0}'.format(process_data[key]))
		if len(points3D) > 0:
			self.Print('# Stereopsis results:')
			for point3D in points3D:
				self.Print('# \t - Point3D: x = {0} \t y = {1} \t z = {2}'.format(point3D[0,0], point3D[1,0], point3D[2,0]))

	#################################### PRINT FUNCTIONS #################################################

	def PrintCalibrationSession(self, calib_folder, stereo_or_blob_calib, calib_filename):
		'''
		 @brief Print start of a new calibration session

		 @param calib_folder (folder where the captured calibration frames are stored)
		 @param stereo_or_blob_calib (True for stereopsis calibration, False for blob scale calibration)
		 @param calib_filename (Basic filename of save frames)
		'''
		if stereo_or_blob_calib:
			self.Print('#----------------- STEREOPSIS CALIBRATION SESSION STARTED -----------------#')
			self.Print('# Stereopsis calibration info:')
			self.Print('# \t Use an appropriate chessboard poster with a shape of {0} rows and {1} columns.'.format(self.__subclass.GetSettings('CALIB', 'calib_chess_rows'), self.__subclass.GetSettings('CALIB', 'calib_chess_columns')))
			self.Print('# \t Aim the cameras at the chessboard so that the whole chessboard fits within the left and right camera frame.')
			self.Print('# \t Take multiple image sets of the chessboard from different angles.')
		else:
			self.Print('#-------------------- BLOB CALIBRATION SESSION STARTED --------------------#')
			self.Print('# Standard distance betwen blobs calibration info:')
			self.Print('# \t Aim the cameras at a blank (non-mirroring) wall.')
			self.Print('# \t Take multiple image sets with structured light projections.')
		self.Print('# Enter designated quit command when appropriate image sets are stored.')
		self.Print('# Designated folder for saving the images: \t ' + calib_folder)
		self.Print('# Designated base filename for the images: \t ' + calib_filename)
		self.Print('#--------------------------------------------------------------------------#')

	def PrintStarting(self, master=False, wait_for_user=True):
		'''
		 @brief Print process information at start.

		 @param master (True/False)
		 @param wait_for_user (True/False)
		'''
		self.Print('#----------------- DRONEVISION STARTING -----------------#')
		self.Print('# Hardware source: \t\t ' + self.__subclass.GetSettings('BASIC', 'source_type'))
		self.PrintStorageInfo()
		if master and wait_for_user:
			self.Print('# Hit enter to begin..')
		self.Print('#--------------------------------------------------------#')
		if master and wait_for_user:
			getpass('')
		self.__start_time 		= timeit.default_timer()
		self.__frame_proc_time 	= timeit.default_timer()

	def PrintProgress(self, process_data=None, points3D=[]):
		'''
		 @brief Print process information during execution.

		 @param process_data (dictionary, {'tag': data}, None if nothing to print (default=None))
		 @param points3D (for printing 3D points (default=[]))
		'''
		elapsed = timeit.default_timer() - self.__start_time
		frame_i  	= self.__subclass.objCameraLink.GetFrameNumber()
		n_frames 	= self.__subclass.objCameraLink.GetTotalFrames()
		self.Print('#----------------- DRONEVISION PROGRESS -----------------#')	
		self.Print('# Execution time: \t\t {0:.2f} sec'.format(elapsed))
		if self.__subclass.GetSettings('BASIC', 'source_type') == 'CAMERA' and self.__subclass.GetSettings('CAMERA', 'n_frames') <= 0:
			self.Print('# Processed frames: \t\t {0}'.format(int(frame_i)))
		else:
			if self.__subclass.GetSettings('CAMERA', 'n_frames') > 0:
				n_frames = self.__subclass.GetSettings('CAMERA', 'n_frames')
			self.Print('# Progress (frame_i/n_frames): \t {0}/{1} - {2:.2f}%'.format(int(frame_i), int(n_frames), (frame_i/n_frames)*100))
			self.Print('# Total frames: \t\t {0}'.format(int(n_frames)))
		if frame_i > 0:
			frame_delay = timeit.default_timer() - self.__frame_proc_time
			self.Print('# Frame processing delay: \t {0:.2f} sec'.format(frame_delay))
			self.Print('# Average delay per frame: \t {0:.2f} sec'.format(elapsed/frame_i))
		self.PrintData(process_data, points3D)
		self.Print('#--------------------------------------------------------#')
		self.__frame_proc_time = timeit.default_timer()

	def PrintFinished(self):
		'''
		 @brief Print process information at finish.
		'''
		elapsed 		= timeit.default_timer() - self.__start_time
		n_proc_frames 	= self.__subclass.objCameraLink.GetFrameNumber()
		self.Print('#----------------- DRONEVISION FINISHED -----------------#')
		self.Print('# Execution time: \t\t {0:.2f}'.format(elapsed))
		self.Print('# Number of frames processed: \t {0}'.format(int(n_proc_frames)))
		if n_proc_frames > 0:
			self.Print('# Average delay per frame: \t {0:.2f} sec'.format(elapsed/n_proc_frames))
		self.PrintStorageInfo()
		self.Print('#--------------------------------------------------------#')

	def PrintStorageInfo(self):
		'''
		 @brief Print storage information
		'''
		self.Print('# Output folder:')
		self.Print('# \t{0}'.format(self.__subclass.GetDatabaseOutputFolder()))
		if self.__subclass.GetSettings('DATABASE', 'store_process_data'):
			self.Print('# Database: \t\t\t {0}'.format(self.__subclass.GetDatabaseName()))
			self.Print('# Database table: \t\t {0}'.format(self.__subclass.GetDatabaseTable()))

	#######################################################################################################################

	def __del__(self):
		'''DESTRUCTOR'''
		self.CloseLog()
