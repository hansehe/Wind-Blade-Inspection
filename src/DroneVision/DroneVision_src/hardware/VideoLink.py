'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import cv2, os

'''
 @brief Class for reading of a video file.

 @param video_filename
 @param sl_video_filename
'''
class VideoLink():
	def __init__(self, video_filename, sl_video_filename):
		'''CONSTRUCTOR'''
		self.__video_filename 		= video_filename
		self.__sl_video_filename 	= sl_video_filename
		self.__frame_i 				= 0
		self.__n_frames 			= 0
		self.__vid_open				= False

		self.OpenVideo()

	def CheckManualTriggering(self):
		'''
		 @brief Check if manual triggering is turned ON

		 @return True/False
		'''
		return False

	def StartCamera(self):
		'''
		 @brief Method for syncronizing with the CameraLink class.
		'''
		self.OpenVideo()

	def OpenVideo(self):
		'''
		 @brief Initialize video 
		'''
		if not(os.path.isfile(self.__video_filename)) or not(os.path.isfile(self.__sl_video_filename)):
			raise Exception('Video file does not exist')
		self.__vidcap 		= cv2.VideoCapture()
		self.__sl_vidcap 	= cv2.VideoCapture()
		self.__vidcap.open(self.__video_filename)
		self.__sl_vidcap.open(self.__sl_video_filename)
		self.__frame_i 		= 0

		if not(self.__vidcap.isOpened()) or not(self.__sl_vidcap.isOpened()):
			raise Exception('Could not open videos')
		if not(self.__vidcap.get(cv2.CAP_PROP_FRAME_COUNT) == self.__sl_vidcap.get(cv2.CAP_PROP_FRAME_COUNT)):
			raise Exception('Number of frame counts for the normal video and the sl video is not consistent.')
		if not(self.__vidcap.get(cv2.CAP_PROP_FPS) == self.__sl_vidcap.get(cv2.CAP_PROP_FPS)):
			raise Exception('FPS for the normal video and the sl video is not consistent.')
		if not(self.__vidcap.get(cv2.CAP_PROP_FRAME_WIDTH) == self.__sl_vidcap.get(cv2.CAP_PROP_FRAME_WIDTH)) or not(self.__vidcap.get(cv2.CAP_PROP_FRAME_HEIGHT) == self.__sl_vidcap.get(cv2.CAP_PROP_FRAME_HEIGHT)):
			raise Exception('Dimensions for the normal video and the sl video is not consistent.')
		
		self.__n_frames 	= self.__vidcap.get(cv2.CAP_PROP_FRAME_COUNT)
		self.__vid_open		= True

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
		fps = self.__vidcap.get(cv2.CAP_PROP_FPS)
		width = self.__vidcap.get(cv2.CAP_PROP_FRAME_WIDTH)
		height = self.__vidcap.get(cv2.CAP_PROP_FRAME_HEIGHT)
		return fps, width, height

	def GetFrame(self, get_normal_frame_only=False):
		'''
		 @brief Pull new frame and structured light (sl) frame from the stored video

		 @param get_normal_frame_only (Only implemented to fit with the CameraLink)

		 @return frame, sl_frame
		'''
		success, frame = self.__vidcap.read()
		if success == False:
			raise TypeError('Cannot pull new frame')

		sl_frame = self.GetSLFrame()

		self.__frame_i += 1
		return frame, sl_frame

	def GetSLFrame(self):
		'''
		 @brief Pull new structured light (SL) frame from the stored video

		 @return sl_frame instance (cv2)
		'''
		success, frame = self.__sl_vidcap.read()
		if success == False:
			raise TypeError('Cannot pull new sl frame')
		return frame

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
		self.StopVideo()

	def StopVideo(self):
		'''
		 @brief Stop and release video instance.
		'''
		if self.__vid_open:
			self.__vidcap.release()
			self.__vid_open = False

	def RestartCamera(self):
		'''
		 @brief Simulating restart
		'''
		pass

	def __del__(self):
		'''
		 @brief Stop video
		'''
		self.StopVideo()