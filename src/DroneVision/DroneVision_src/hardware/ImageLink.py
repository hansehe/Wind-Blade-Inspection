'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''
import os, glob
from imageTools import GetImage
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape

'''
 @brief Set up image linke.

 @param folder (folder where the files are located)
 @param image_filenames (Single or multiple (list) images without structured light)
 @param sl_image_filenames (Single or multiple (list) images with structured light)
'''
class ImageLink():
	def __init__(self, folder, image_filenames, sl_image_filenames):
		'''CONSTRUCTOR'''
		self.CheckFilenames(folder, image_filenames, sl_image_filenames)
		self.OpenImage()

	def CheckManualTriggering(self):
		'''
		 @brief Check if manual triggering is turned ON

		 @return True/False
		'''
		return False

	def CheckFilenames(self, folder, image_filenames, sl_image_filenames):
		'''
		 @brief Check if filenames are list of files, a directory with consistent files, or a single file.

		 @param image_filenames
		 @param sl_image_filenames
		'''
		if isinstance(image_filenames, list) or isinstance(sl_image_filenames, list):
			if not(isinstance(image_filenames, list) and isinstance(sl_image_filenames, list)):
				raise Exception('Both sl filenams and normal filenams must be a list')
			self.__image_filenames 		= image_filenames
			self.__sl_image_filenames 	= sl_image_filenames
		else:
			if (os.path.isdir(folder + image_filenames) or os.path.isdir(folder + sl_image_filenames)) or (image_filenames[-1:][0] == '/' or sl_image_filenames[-1:][0] == '/'):
				image_filenames 	= folder + image_filenames
				sl_image_filenames 	= folder + sl_image_filenames
				folder = ''
				if not(os.path.isdir(image_filenames) and os.path.isdir(sl_image_filenames)):
					raise Exception('Both sl filenams and normal filenams must be a directory')
				self.__image_filenames 		= glob.glob(image_filenames+'/*.*')
				self.__sl_image_filenames 	= glob.glob(sl_image_filenames+'/*.*')
			else:
				self.__image_filenames 		= [image_filenames]
				self.__sl_image_filenames 	= [sl_image_filenames]
		self.__folder = folder

	def StartCamera(self):
		'''
		 @brief Method for syncronizing with the CameraLink class.
		'''
		self.OpenImage()

	def OpenImage(self):
		'''
		 @brief Initialize video 
		'''
		self.__frame_i 	= 0
		self.__n_frames = len(self.__image_filenames)
		self.GetFrame() # Get frames for property possibilities
		self.__frame_i 	= 0 #reset

	def GetTotalFrames(self):
		'''
		 @brief Get total frames in video

		 @return n_frames
		'''
		return self.__n_frames

	def GetFrameProperties(self):
		'''
		 @brief Get frame properties such as fps, width, length

		 @return fps, width, height
		'''
		dim = GetShape(self.__frame)
		return 1.0, dim[0], dim[1]

	def GetFrame(self, get_normal_frame_only=False):
		'''
		 @brief Pull image and structured light image

		 @param get_normal_frame_only (Only implemented to fit with the CameraLink)

		 @return frame, sl_frame
		'''
		self.__frame 	= GetImage(self.__folder + self.__image_filenames[self.__frame_i])
		self.__sl_frame = GetImage(self.__folder + self.__sl_image_filenames[self.__frame_i])

		if not(GetShape(self.__frame)[0] == GetShape(self.__sl_frame)[0]) or not(GetShape(self.__frame)[1] == GetShape(self.__sl_frame)[1]):
			raise Exception('Normal image and sl image dimensions are not consistent.')

		self.__frame_i += 1
		return self.__frame, self.__sl_frame

	def GetFrameNumber(self):
		'''
		 @brief Get current frame number

		 @return frame_i
		'''
		return self.__frame_i

	def StopCamera(self):
		'''
		 @brief Method for syncronizing with the CameraLink class.
		'''
		self.StopImage()

	def StopImage(self):
		'''
		 @brief Stop image (do nothing actually..)
		'''
		pass

	def RestartCamera(self):
		'''
		 @brief Simulating restart
		'''
		pass

	def __del__(self):
		'''
		 @brief Stop video
		'''
		self.StopImage()