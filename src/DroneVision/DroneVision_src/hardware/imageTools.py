'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

from src.bin.tools import CheckDir
import cv2, math, os, sys, warnings
import numpy as np
import matplotlib

def CheckDisplayAvailable(linux_embedded_name='posix'):
	'''
	 @brief Check if display is available

	 @param linux_embedded_name (default='posix')

	 @return True/False
	'''
	available = True
	if os.name == linux_embedded_name:
		if os.environ.get('DISPLAY') is None:
			available = False
	return available

if not(CheckDisplayAvailable()):
   	matplotlib.use('Agg')
#import matplotlib.pyplot as plt

def GetImage(image_filename, gray=True):
	'''
	 @brief Get the frame from the input image filename.

	 @param image_filename Image filename
	 @param gray Return image as grayscale (default = True), or 'as is' (gray = False).

	 @return frame (grayscale)
	'''
	if not(os.path.isfile(image_filename)):
		raise Exception('Image file does not exist: {0}'.format(image_filename))
	if gray:
		frame = cv2.imread(image_filename, cv2.IMREAD_GRAYSCALE)
	else:
		frame = cv2.imread(image_filename)
	if not(isinstance(frame, np.ndarray)):
		raise ValueError('Frame was not loaded properly!')
	return frame

def WriteImage(frame, image_output_filename):
	'''
	 @brief Write the frame with the output filename using .tif format

	 @param frame Frame to write
	 @param image_output_filename Image filename
	'''
	if not(frame.dtype == np.uint8 or frame.dtype == np.uint16):
		frame = frame.astype(np.uint16)
	cv2.imwrite(image_output_filename+'.tif', frame)

def MultiWriteImage(dict_frames):
	'''
	 @brief Write multiple images

	 @param dict_frames Dictionary of frames - example: mult_frames = {'img_name1': frame1, 'img_name2': frame2}
	'''
	for key in dict_frames:
		WriteImage(dict_frames[key], key)

def ImShow(frame, image_name):
	'''
	 @brief Show frame and wait for user input (any key) to continue.

	 @param frame Frame to show
	 @param image_name Nickname of image
	'''
	if not(CheckDisplayAvailable()):
		WriteImage(frame, image_name)
	else:
		cv2.imshow(image_name, frame)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

def MultiImShow(dict_frames):
	'''
	 @brief Show multiple images and wait for user input (any key) to continue.

	 @param dict_frames Dictionary of frames - example: mult_frames = {'img_name1': frame1, 'img_name2': frame2}
	'''
	if not(CheckDisplayAvailable()):
		MultiWriteImage(dict_frames)
	else:
		for key in dict_frames:
			cv2.imshow(key, dict_frames[key])
		cv2.waitKey(0)
		cv2.destroyAllWindows()

def MatplotShow(touple_frames, main_title='', fig_number=0, interactive_mode=False, plt=None, savefig_folder='matplot_figs/', default_n_cols=2, save_fig=False, save_fig_only=False, inlude_main_title_in_plot=True):
	'''
	 @brief Show multiple grayscale frames in a single window.

	 @param touple_frames Touple of frames - example: mult_frames = [('img_name1', frame1), ('img_name2', frame2)]
	 @param main_title (default='')
	 @param fig_number (>= 0, to handle more than one figure (default=0))
	 @param interactive_mode (Flag to handle interactive mode (True/False - default=False)))
	 @param plt (pyplot object from matplotlib (default=None))
	 @param savefig_folder (Folder path for saving figures when display isn't available.)
	 @param default_n_cols (default number of columns (default=2))
	 @param save_fig (Save figure (default=False))
	 @param save_fig_only (Save figure, and do not show plot (default=False))
	 @param inlude_main_title_in_plot (Include main title in plot. Also used as a filename for saving.)
	'''
	if plt == None:
		import matplotlib.pyplot as plt
	n_frames = len(touple_frames)
	cols = default_n_cols
	if n_frames == 0:
		return
	elif n_frames < cols:
		cols = n_frames
	rows = int(math.ceil(float(n_frames)/cols))
	fig = plt.figure(fig_number)
	i = 1
	for img_name, frame in touple_frames:
		ax = fig.add_subplot(rows, cols, i)
		if len(frame.shape) == 2:
			plt.imshow(frame, cmap='gray')
		else:
			plt.imshow(frame)
		ax.set_title(img_name)
		i += 1
	if inlude_main_title_in_plot:
		fig.suptitle(main_title)
	fig.tight_layout()
	
	if not(CheckDisplayAvailable()) or save_fig or save_fig_only:
		CheckDir(savefig_folder)
		main_title = os.path.basename(main_title) # Does not change a normal title, just removes all paths
   		fig.savefig(savefig_folder+main_title+'.png')

   	if not(save_fig_only):
	   	if not(interactive_mode):
	   		if CheckDisplayAvailable():
	   			print 'Press Esc on figure plot to continue..'
	   		try:
				plt.show()
			except Exception, err:
				warnings.simplefilter('always')
				warnings.warn(str(err), Warning)
				warnings.simplefilter('default')
		else:
			try:
				plt.pause(0.000001)
			except Exception, err:
				warnings.simplefilter('always')
				warnings.warn(str(err), Warning)
				warnings.simplefilter('default')

'''
 @brief Simple real time plot class

 @param touple frames Touple of frames - example: mult_frames = [('img_name1', frame1), ('img_name2', frame2)]
 @param main_title (default='')
 @param fig_number (>= 0, to handle more than one figure (default=0))
 @param interactive_mode (True/False for flagging interactive mode (plot figure is non-blocking and stays open) (default=True))
 @param savefig_folder (Folder path for saving figures when display isn't available.)
 @param default_n_cols (default number of columns (default=2))
 @param save_fig (Save figure (default=False))
 @param save_fig_only (Save figure, and do not show plot (default=False))
 @param inlude_main_title_in_plot (Include main title in plot. Also used as a filename for saving.)
'''
class RealTimePlot():
	def __init__(self, touple_frames=[], main_title='', fig_number=0, interactive_mode=True, savefig_folder='matplot_figs/', default_n_cols=2, save_fig=False, save_fig_only=False, inlude_main_title_in_plot=True):
		'''CONSTRUCTOR'''
		import matplotlib.pyplot as plt
		self.__plt 							= plt
		self.__interactive_mode 			= interactive_mode
		self.__fig_number 					= fig_number
		self.__main_title 					= main_title
		self.__savefig_folder 				= savefig_folder
		self.__default_n_cols 				= default_n_cols
		self.__save_fig 					= save_fig
		self.__save_fig_only 				= save_fig_only
		self.__inlude_main_title_in_plot 	= inlude_main_title_in_plot
		self.__n_plotted_frames 			= 0
		self.__n_plot 						= 0
		if self.__interactive_mode and CheckDisplayAvailable():
			self.__plt.ion()
		self.__call__(touple_frames, main_title)

	def SetSaveFigFolder(self, savefig_folder):
		'''
		 @brief Set save fig folder
		'''
		self.__savefig_folder = savefig_folder

	def __call__(self, touple_frames=[], main_title=None, reset=False):
		'''
		 @brief Call function

		 @param touple_frames
		 @param main_title
		 @param reset (default=False)
		 @param empty_plot Set an empty default plot if the plot were reset (default=True).
		'''
		#self.__plt.gcf().clear()
		if reset or (len(touple_frames) != self.__n_plotted_frames and self.__n_plotted_frames > 0):
			self.__n_plot = 0
			self.__plt.figure(self.__fig_number).clf()
		if main_title == None:
			main_title = self.__main_title
		main_title_n = main_title
		if not(CheckDisplayAvailable()):
			main_title_n += '_'+str(self.__n_plot)
		if len(touple_frames) > 0:
			self.__n_plotted_frames = len(touple_frames)
			MatplotShow(touple_frames, main_title_n, self.__fig_number, self.__interactive_mode, self.__plt, self.__savefig_folder+main_title+'/', self.__default_n_cols, self.__save_fig, self.__save_fig_only, self.__inlude_main_title_in_plot)
			self.__n_plot += 1

	def __del__(self):
		'''DESTRUCTOR'''
		self.__plt.close('all')
		self.__plt.ioff()