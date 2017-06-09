'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision)
'''

import cv2, random, warnings
import numpy as np

def GetRandomColor(sum_threshold=10, max_steps=10, step=0):
	'''
	 @brief Get random color

	 @param sum_threshold (minimum sum of colors (default=10))
	 @param max_steps (max steps on finding a random color (default=10))

	 @return color (r,g,b)
	'''
	r = random.randrange(0,256)
	g = random.randrange(0,256)
	b = random.randrange(0,256)
	if r+g+b < sum_threshold:
		if step > max_steps:
			raise Exception('Too many steps finding a random color')
		return GetRandomColor(sum_threshold=sum_threshold, max_steps=max_steps, step=step+1)
	return (r,g,b)

def GetShape(frame):
	'''
	 @brief Get shape of frame, regardless of depth dimension.

	 @param frame 
	 
	 return (height, width)
	'''
	return frame.shape[:2]

def ToGrayscale(frame):
	'''
	 @brief Convert color frame to grayscale.

	 @param frame (color)
	 
	 @return grayscale frame
	'''
	return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

def ToColor(frame):
	'''
	 @brief Convert grayscale frame to color.

	 @param frame (grayscale)
	 
	 @return color frame
	'''
	return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

def CheckGrayScale(frame):
	'''
	 @brief Check wether the frame is of grayscale or color.
	 	converts the frame to grayscale if necessary.

	 @param frame
	 
	 @return grayscale frame.
	'''
	shape = frame.shape
	if len(shape) == 3:
		gray_frame = ToGrayscale(frame)
	else:
		gray_frame = frame
	return gray_frame

def CheckColor(frame):
	'''
	 @brief Check wether the frame is of grayscale or color.
	 	converts the frame to color if necessary.

	 @param frame
	 
	 @return color frame.
	'''
	shape = frame.shape
	if len(shape) == 2:
		cl_frame = ToColor(frame)
	else:
		cl_frame = frame
	return cl_frame

def FilterByColor(frame, lower=None, upper=None, hvs_cl_threshold=0):
	'''
	 @brief Filter color frame by color

	 @param frame (color frame)
	 @param lower (Lower boundary given as tuple of (B, G, R))
	 @param upper (Upper boundary given as tuple of (B, G, R))
		Upper and lower limits are set to detect green colors if either is None

	 @return mask (Filter mask, nxm array of masked values)
	'''
	if isinstance(lower, type(None)) or isinstance(upper, type(None)):
		lower 		= (60 - hvs_cl_threshold, 40, 200)
		upper 		= (60 + hvs_cl_threshold, 255, 255)
	hsv_frame 	= cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	mask 		= cv2.inRange(hsv_frame, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
	return mask

def CropFrame(frame, crop_frame_divisor):
	'''
	 @brief Crop frame around the frame center according to the crop frame divisor

	 @param frame (grayscale)
	 @param crop_frame_divisor (0 < Float <= 1 - divisor for cropping frames. F.ex 0.5 will crop the frame to half the size around the frame center.)
	
	 @return cropped_frame
	'''
	height, width = GetShape(frame)
	center_height 	= height/2.0
	center_width	= width/2.0
	crop_c_height 	= center_height*crop_frame_divisor
	crop_c_width 	= center_width*crop_frame_divisor
	cropped_frame 	= frame[int(center_height - crop_c_height):int(center_height + crop_c_height), int(center_width - crop_c_width):int(center_width + crop_c_width)]
	return cropped_frame

def ComputePyrDownDivisor(desired_frame_shape, incoming_frame_shape):
	'''
	 @brief Compute downsample divisor by referring to a desired frame shape, and the shape of incoming frames.

	 @param desired_frame_shape (Desired shape of processed frames, given as a tuple of (height, width)).
	 @param desired_frame_shape (Original shape of incoming frames, given as a tuple of (height, width) - (preferrably use GetShape(origin_frame))).

	 @return pyr_down_divisor (int)
	'''
	h_div = incoming_frame_shape[0]//desired_frame_shape[0]
	w_div = incoming_frame_shape[1]//desired_frame_shape[1]
	if w_div != h_div:
		raise ValueError('Desired (height, width): {0} did not result in matching divisors when comparing to the incoming frames of shape (height, width): {1}!'.format(desired_frame_shape, incoming_frame_shape))
	if w_div < 1:
		warnings.warn('Desired frames shape is bigger than incoming frame shapes - setting pyrdown divisor to 1 (no downsampling).', Warning)
		pyr_down_divisor = 1
	else:
		pyr_down_divisor = w_div
	return pyr_down_divisor

def PyrDown(frame, default_divisor=2, desired_frame_shape=(-1,-1)):
	'''
	 @brief Downsample frame using gaussian pyramid downsampling.

	 @param frame
 	 @param default_divisor - Default level of downsampling (default=2)
	 @param desired_frame_shape - Desired final frame shape, given as a tuple of (height, width). Set to (-1,-1) to not compute a divisor, but stay to the fixed default_divisor (default=(-1,-1))
	 	- args in cv2 method:
	 		1. src frame
	 		2. dst frame
	 		2. new dimension
	
	 @return filtered frame
	'''
	if desired_frame_shape[0] <= 0:
		divisor = default_divisor
	else:
		divisor = ComputePyrDownDivisor(desired_frame_shape, GetShape(frame))
	delimiter = 2
	n_divisor = 1
	while n_divisor < divisor:
		#PP: Reduce image according to gaussian pyramid method
		rows, cols 	= GetShape(frame)
		frame 		= cv2.pyrDown(frame, frame, (cols/delimiter, rows/delimiter))
		n_divisor 	*= delimiter
	return frame