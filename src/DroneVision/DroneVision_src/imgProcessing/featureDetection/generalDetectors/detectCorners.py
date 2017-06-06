'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import cv2
import numpy as np
from Settings.Exceptions import DroneVisionError
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import GetShape, CheckColor

def DetectCorners(frame, keypoints=[], kernel_size=5, kernel_step_size=1, k=0.04, thresh=1000):
	'''
	 @brief Detect corners using the Harris corner method.
	 	Inspired by: https://github.com/hughesj919/HarrisCorner/blob/master/Corners.py

	 @param frame (grayscale)
	 @param keypoints (list of cv2 keypoints. The frame will be reset if any keypoints are given. (default=empty list))
	 @param kernel_size (It is the size of neighbourhood considered for corner detection (default=5))
	 @param kernel_step_size (default=1)
	 @param k (Harris corner constant - usually between 0.04 - 0.06 (default=0.04))
	 @param thresh (Response threshold (default=1000))

	 @return corners (list of corners given as cv2 keypoints. Get position of keypoint as kp.pt (x,y position) and response of corner as kp.response)
	'''
	corners 			= []
	offset 				= int(kernel_size//2)
	kernel_step_size 	= int(round(kernel_step_size))

	len_kp = len(keypoints)
	if len_kp > 0: # Find corners from keypoints.
		corners_frame = np.zeros(GetShape(frame), dtype=np.uint8)
		for i in range(len_kp):
			corners_frame[int(round(keypoints[i].pt[1])), int(round(keypoints[i].pt[0]))] = keypoints[i].size
	else: # Find corners in given frame.
		corners_frame = frame

	#Find x and y derivatives
	dy, dx = np.gradient(corners_frame)
	Ixx = dx**2
	Ixy = dy*dx
	Iyy = dy**2
	height, width = GetShape(corners_frame)

	for y in range(offset, height-offset, kernel_step_size):
		for x in range(offset, width-offset, kernel_step_size):
			#Calculate sum of squares
			windowIxx = Ixx[y-offset:y+offset+1, x-offset:x+offset+1]
			windowIxy = Ixy[y-offset:y+offset+1, x-offset:x+offset+1]
			windowIyy = Iyy[y-offset:y+offset+1, x-offset:x+offset+1]
			Sxx = windowIxx.sum()
			Sxy = windowIxy.sum()
			Syy = windowIyy.sum()

			#Find determinant and trace, use to get corner response
			det = (Sxx * Syy) - (Sxy**2)
			trace = Sxx + Syy
			r = det - k*(trace**2)

			#If corner response is over threshold, add to corner list
			if r > thresh:
				corners.append(cv2.KeyPoint(x=x, y=y, _size=r/thresh, _response=r))
	return corners

def DetectCornersCV2(frame, keypoints=[], kernel_size=5, k_sobel_size=3, k=0.04, thresh=0.8):
	'''
	 @brief Detect corner using the Harris corner detector as provided by the opencv library.

	 @param frame (grayscale)
	 @param keypoints (list of cv2 keypoints. The frame will be reset if any keypoints are given. (default=empty list))
	 @param kernel_size (It is the size of neighbourhood considered for corner detection (default=5))
	 @param k_sobel_size (Aperture parameter of Sobel derivative used. (default=3))
	 @param k (Harris corner constant - usually between 0.04 - 0.06 (default=0.04))
	 @param thresh (Threshold value relative to the maximum corner response. (default=0.01))

	 @return corners (list of corners given as cv2 keypoints. Get position of keypoint as kp.pt (x,y position) and response of corner as kp.response)
	'''
	len_kp = len(keypoints)
	if len_kp > 0: # Find corners from keypoints.
		corners_frame = np.zeros(GetShape(frame), dtype=np.float32)
		for i in range(len_kp):
			corners_frame[int(round(keypoints[i].pt[1])), int(round(keypoints[i].pt[0]))] = keypoints[i].size
	else: # Find corners in given frame.
		corners_frame = np.float32(frame)

	corners_frame = cv2.cornerHarris(corners_frame, kernel_size, k_sobel_size, k)

	#result is eroded for marking the corners
	corners_frame = cv2.erode(corners_frame, np.ones((3,3), dtype=np.uint8))

	# Threshold for an optimal value, it may vary depending on the image.
	kp_positions 	= np.flip(np.argwhere(corners_frame > thresh*corners_frame.max()), 1)
	n_kps 			= len(kp_positions)
	corners 		= [cv2.KeyPoint()]*n_kps
	for i in range(n_kps):
		corners[i].pt 		= tuple(kp_positions[i])
		corners[i].size 	= corners_frame[kp_positions[i,1], kp_positions[i,0]]
		corners[i].response = corners_frame[kp_positions[i,1], kp_positions[i,0]]
	return corners

def DrawCorners(frame, corners, size_delimiter_param=1.0, color=(255,0,0)):
	'''
	 @brief Draw corners

	 @param frame
	 @param corners (list of keypoint corners)
	 @param size_delimiter_param (default=1.0)
	 @param color (default=(255,0,0))

	 @return frame (color)
	'''
	frame = CheckColor(frame)
	for corner in corners:
		y 		= int(round(corner.pt[1]))
		x 		= int(round(corner.pt[0]))
		#radius 	= int(round(corner.size/size_delimiter_param))
		radius 	= 10
		cv2.circle(frame, (x, y), radius, color, 3)
	return frame