'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import cv2
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape

'''
 @brief PyQtImage class.
 	Real-time plot of images

 @param initialize (True/False for initializing PQ graph in constructor (default=True))
 @param title (Title of real-time graph)
'''
class PyQtImage():
	def __init__(self, initialize=True, title='Real-time Plot'):
		'''CONSTRUCTOR'''
		self.__win_open 		= False
		self.__flag_reset 		= False
		self.__title 			= title
		self.__n_plotted_frames = 0
		if initialize:
			self.InitPQImage(title=title)

	def InitPQImage(self, title='Real-time Plot'):
		'''
		 @brief Initialize PyQtImage

		 @param title
		'''
		if not(self.__win_open):
			import pyqtgraph as pg
			self.__pg 		= pg
			self.__title 	= title

			self.__win 		= self.__pg.GraphicsWindow(title=self.__title)
			self.__win.resize(1000,600)

			self.__img_items 	= {}
			self.__view_boxes 	= {}
			self.__win_open 	= True

	def AssertPQImageInititalized(self):
		'''
		 @brief Assert that the PQ image plot is initialized
		'''
		if not(self.__win_open):
			raise Exception('PqtGraph is not initialized. Run "InitPQImage"')

	def ResetPQImagePlot(self):
		'''
		 @brief Reset pqt plot
		'''
		if self.__win_open:
			self.__win.clear()
			self.__img_items 	= {}
			self.__view_boxes 	= {}
			self.TriggProcessEvents()
			self.__flag_reset = True

	def CheckFlagReset(self):
		'''
		 @brief Check flag reset for triggering a second time since pyqthgraph has some issues with resetting

		 @return True/False
		'''
		flag_reset = self.__flag_reset
		self.__flag_reset = False
		return flag_reset

	def AddPQImagePlot(self, key_tag):
		'''
		 @brief Add image plot
		
		 @param key_tag
		'''
		self.AssertPQImageInititalized()
		self.__img_items[key_tag] 	= self.__pg.ImageItem(name=key_tag)
		self.__view_boxes[key_tag] 	= self.__pg.ViewBox(name=key_tag)
		self.__view_boxes[key_tag] 	= self.__win.addViewBox()
		self.__view_boxes[key_tag].addItem(item=self.__img_items[key_tag])

	def UpdatePQImage(self, frame, key_tag, process_events=True, rotate=True, autoDownsample=True):
		'''
		 @brief Plot image in real-time

		 @param frame
		 @param key_tag (also frame title)
		 @param process_events (True/False for processing new events (plotting new data))
		 @param rotate (True/False for rotating the image so that it is shown properly. (default=True))
		 @param autoDownsample (True/False (default=True))
		'''
		if not(self.__win_open):
			self.InitPQImage(title=self.__title)
		self.AssertPQImageInititalized()
		if not(key_tag in self.__img_items):
			self.AddPQImagePlot(key_tag)
		if rotate:
			width, height = GetShape(frame)
			M = cv2.getRotationMatrix2D((height/2, width/2), -90, 1)
			self.__img_items[key_tag].setImage(cv2.warpAffine(frame, M, (height,width)), autoDownsample=autoDownsample)
		else:
			self.__img_items[key_tag].setImage(frame, autoDownsample=autoDownsample)
		if process_events:
			self.TriggProcessEvents()
			if self.CheckFlagReset():
				self.UpdatePQImage(frame, key_tag, process_events, rotate, autoDownsample)

	def UpdatePQImages(self, touple_frames, rotate=True):
		'''
		 @brief Update a set of frames
		 
	 	 @param touple_frames Touple of frames - example: mult_frames = [('img_name1', frame1), ('img_name2', frame2)]
	 	 @param rotate (True/False for rotating the image so that it is shown properly. (default=True))
		'''
		if not(self.__win_open):
			self.InitPQImage(title=self.__title)
		for touple_frame in touple_frames:
			self.UpdatePQImage(touple_frame[1], touple_frame[0], process_events=False, rotate=rotate)
		self.TriggProcessEvents()
		if self.CheckFlagReset():
			self.UpdatePQImages(touple_frames, rotate)

	def TriggProcessEvents(self):
		'''
		 @brief Trigg new processing of events.
		'''
		self.__pg.QtGui.QApplication.processEvents()

	def ClosePQWindow(self):
		'''
		 @brief Close and end this window
		'''
		if self.__win_open:
			self.__win.close()
			self.__win_open = False

	def __call__(self, touple_frames=[], rotate=True, reset=False):
		'''
		 @brief Update a set of frames
		 
	 	 @param touple_frames Touple of frames - example: mult_frames = [('img_name1', frame1), ('img_name2', frame2)]
	 	 @param rotate (True/False for rotating the image so that it is shown properly. (default=True))
	 	 @param reset (Reset pygt plot. Input touple frames are ignored. (default=False))
		'''
		if reset or (len(touple_frames) != self.__n_plotted_frames and self.__n_plotted_frames > 0):
			self.ResetPQImagePlot()
		if len(touple_frames) > 0:
			self.__n_plotted_frames = len(touple_frames)
			self.UpdatePQImages(touple_frames, rotate)

   	def __del__(self):
   		'''DESTRUCTOR'''
   		self.ClosePQWindow()
