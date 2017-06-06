'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''
import numpy as np
from src.bin.tools import CheckDir
from src.DroneVision.DroneVision_src.hardware.RecordFrames import RecordFrames

'''
 @brief FrameRecorder class for recording frames.

 @param default_fps (frames per second default value (default=1.0))
 @param store_frames_as_video (store all recorded frames as video (default=True))
 @param store_frames_as_images (store all recorded frames as images in separate subfolder (default=True))
'''
class FrameRecorder():
	def __init__(self, default_fps=1.0, store_frames_as_video=True, store_frames_as_images=True):
		'''CONSTRUCTOR'''
		self.__frame_recorders_dict 		= {}
		self.__fps 							= default_fps
		self.__store_frames_as_video		= store_frames_as_video
		self.__store_frames_as_images 		= store_frames_as_images
		self.ResetProcessFrames()

	def ResetProcessFrames(self):
		'''
		 @brief Reset process frames by deleting all values
		'''
		self.__process_frames = {}

	def SetProcessFrame(self, tag, frame):
		'''
		 @brief Set frames dictionary for storing frame in the database 

		 @param tag
		 @param frame
		'''
		self.__process_frames[tag] = frame

	def GetProcessFrames(self):
		'''
		 @brief Get process frames dictionary
		
		 @return process_frames
		'''
		return self.__process_frames

	def InitFrameRecorder(self, folder, fps=None):
		'''
		 @brief Initialize frame recorders

		 @param folder
		 @param fps (If None, then it is set by default (default=None))
		'''
		self.__output_folder = folder + 'recordings/'
		if not(fps == None):
			self.__fps = fps

	def RecordProcessFrames(self):
		'''
		 @brief Record process frames
		'''
		CheckDir(self.__output_folder)
		for key in self.__process_frames:
			if not(key in self.__frame_recorders_dict):
				self.__frame_recorders_dict[key] = self.CreateFrameRecorder(self.__fps, self.__output_folder, key, self.__store_frames_as_video, self.__store_frames_as_images)
			self.RecordFrame(self.__frame_recorders_dict[key], self.__process_frames[key])
			
	def RecordFrame(self, frame_recorder, frame, use_threading=False):
		'''
		 @brief Record frame

		 @param frame_recorder
		 @param frame
		 @param use_threading (True/False (default=False))
		'''
		if isinstance(frame, np.ndarray):
			if use_threading:
				frame_recorder.WriteFrameThread(frame)
			else:
				frame_recorder.WriteFrame(frame)

	def CreateFrameRecorder(self, fps, output_folder, video_name, store_frames_as_video=True, store_frames_as_images=True):
		'''
		 @brief Create new frame recorder

		 @param fps (frames per second)
		 @param video_name (video filename)
		 @param store_frames_as_video (store all recorded frames as video (default=True))
		 @param store_frames_as_images (store all recorded frames as images in separate subfolder (default=True))

		 @return recordFrames (class instance of RecordFrames)
		'''
		recordFrames = RecordFrames(fps, output_folder, video_name, store_frames_as_video, store_frames_as_images)
		return recordFrames

	def CloseRecordings(self):
		'''
		 @brief Safely close all recordings
		'''
		for key in self.__frame_recorders_dict:
			self.__frame_recorders_dict[key].CloseRecording()

	def __del__(self):
		'''DESTRUCTOR'''
		self.CloseRecordings()

