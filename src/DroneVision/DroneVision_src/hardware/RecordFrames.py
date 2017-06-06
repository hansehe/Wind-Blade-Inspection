'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''
import cv2, threading
import numpy as np

from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape, CheckColor
from imageTools import WriteImage
from src.bin.tools import CheckDir, RunThread

'''
 @brief Class for frame recording

 @param fps Frames per second
 @param video_output_filename Filename for the output video
 @param store_frames_as_video Store frames as a video (True/False (default=True))
 @param store_frames_as_images Store each frame as an image in a subfolder 'frames' (True/False (default=True))
'''
class RecordFrames():
	def __init__(self, fps, folder, video_output_filename, store_frames_as_video=True, store_frames_as_images=True):
		'''CONSTRUCTOR'''
		self.__folder 					= folder
		self.__video_output_filename 	= video_output_filename
		self.__fps 						= int(np.ceil(fps))
		self.__frame_i 					= 0
		self.__store_frames_as_video 	= store_frames_as_video
		self.__store_frames_as_images 	= store_frames_as_images
		self.__write_lock 				= threading.Lock()

	def GetNumberOfRecordedFrames(self):
		'''
		 @brief Get number of recorded frames

		 @return number of recorded frames
		'''
		return self.__frame_i

	def InitRecording(self, initFrame):
		'''
		 @brief Initialize recording by adapting the video to the first frame input.

		 @param initFrame
		'''
		self.__height, self.__width = GetShape(initFrame)
		self.StartRecording()

	def StartRecording(self):
		'''
		 @brief Start video recording
		 	Creates a folder for the video, called by the same video filename,
		 	and creates a subfolder for storing each frame.
		'''
		# Create frames folder
		if self.__store_frames_as_images:
			self.__frames_folder = self.__folder + self.__video_output_filename+'_frames/'
			CheckDir(self.__frames_folder)

		if self.__store_frames_as_video:
			# Define the codec and create VideoWriter object
			fourcc = cv2.VideoWriter_fourcc(*'MJPG')
			self.video_out = cv2.VideoWriter(filename=self.__folder + self.__video_output_filename+'.avi', fourcc=fourcc, fps=self.__fps, frameSize=(self.__width, self.__height), isColor=True)
		
		self.__frame_i = 0

	def WriteFrameThread(self, frame):
		'''
		 @brief Write frame in thread for smoother program flow.
		'''
		RunThread(self.WriteFrame, args=(frame,), lock=self.__write_lock, wait_lock=True)

	def WriteFrame(self, frame):
		'''
		 @brief Write new frame to output video

		 @param frame Frame to write
		'''
		with self.__write_lock:
			if self.__frame_i == 0:
				self.InitRecording(frame)
			if not(frame.dtype == np.uint8 or frame.dtype == np.uint16):
				frame = frame.astype(np.uint16)
			if self.__store_frames_as_video:
				self.video_out.write(CheckColor(frame))
			if self.__store_frames_as_images:
				WriteImage(frame, self.__frames_folder + self.__video_output_filename + '_' + str(self.__frame_i))
			self.__frame_i += 1

	def CloseRecording(self):
		'''
		 @Stop video recording
		'''
		if self.__frame_i > 0:
			if self.__write_lock.locked():
				pass
			if self.__store_frames_as_video:
				self.video_out.release()

	def __del__(self):
		'''DESTRUCTOR'''
		self.CloseRecording()