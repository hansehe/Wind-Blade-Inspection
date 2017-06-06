'''
 Author: Hans Erik Heggem
 Email: hans.erik.heggem@gmail.com
 Project: Master's Thesis - Autonomous Inspection Of Wind Blades
 Repository: Master's Thesis - CV (Computer Vision
'''

import math, cv2, operator
import numpy as np
from Settings.Exceptions import DroneVisionError
from src.DroneVision.DroneVision_src.imgProcessing.frameTools.frameTools import CheckGrayScale, CheckColor, GetShape
from src.DroneVision.DroneVision_src.hardware.imageTools import MatplotShow

def FindLineLimits(frame, hough_lines, keypoints, radi_threshold=None, radi_threshold_tuning_param=2.0, draw_hough_matrix=False, draw_bounded_lines=False, draw_max_min_lines=False, draw_arrowed_bounded_lines=True):
	'''
	 @brief Find boundaries for all hough lines according to the point map.
	 	Segments an object based on the point list by limiting the hough lines and detecting the boundary lines.

	 @param frame (grayscale)
	 @param hough_lines List of hough lines (rho, theta).
	 @param keypoints List of detected point positions.
	 @param radi_threshold Threshold in pixels to search for in vertical and horizontal axis (If none, then it is set to the biggest blob size)
	 @param radi_threshold_tuning_param Threshold tuning parameter (default=2 when using biggest blob size. < 0.5 is recommended when using distance betwen blobs).
	 @param draw_hough_matrix Draw hough lines matrix.
	 @param draw_bounded_lines Draw bounded lines.
	 @param draw_max_min_lines Draw detected max min lines.
	 @param draw_arrowed_bounded_lines

	 @return frame, bounded_lines, max_hor_line, min_hor_line, max_vert_line, min_vert_line 
	 	frame - Usually grayscale, but rgb frame if any draw flag == True
	 	bounded_lines - list with vertical and horizontal lines. 
	 					bounded_lines[0] = horizontal lines list, bounded_lines[1] = vertical lines list, 
	 					where each index describing a line as ((x1,y1), (x2,y2))
	 	max_min_lines - List of max and min horizontal and vertical lines, in this order: 
		 	max_hor_line - Bottom boundary line on the horizontal axis (Note: bottom = max index in images) as ((x1,y1), (x2,y2))
		 	min_hor_line - Top boundary line on the horizontal axis (Note: top = min index in images) as ((x1,y1), (x2,y2))
		 	max_vert_line - Right most boundary line on the vertical axis as ((x1,y1), (x2,y2))
		 	min_vert_line - Left most boundary line on the horizontal axis as ((x1,y1), (x2,y2))
	'''

	x_points = np.zeros(len(keypoints)) #Width
	y_points = np.zeros(len(keypoints)) #height
	biggest_size = 0.0
	for i in range(len(keypoints)): 
		if keypoints[i].size > biggest_size:
			biggest_size = keypoints[i].size
		x_points[i] = keypoints[i].pt[0]
		y_points[i] = keypoints[i].pt[1]

	# Tuning variable - search along vertical or horizontal axis is limited to this radius.
	if radi_threshold == None:
		radi_threshold = biggest_size
	radi_threshold *= radi_threshold_tuning_param

	min_vert_points = [[],[]]
	max_vert_points = [[],[]]
	min_hor_points 	= [[],[]]
	max_hor_points 	= [[],[]]
	bounded_lines 	= [[],[]]
	width, height 	= GetShape(frame)

	size = math.sqrt(math.pow(width, 2.0) + math.pow(height, 2.0))
	max_vert_line_y_pos = -size*10 # Set invalid 'large' values to begin with
	min_vert_line_y_pos = size*10
	max_hor_line_x_pos 	= -size*10
	min_hor_line_x_pos 	= size*10

	if draw_hough_matrix:
		frame = DrawHoughLines(frame, hough_lines)

	for rho, theta in hough_lines:
		a 	= np.cos(theta)
		b 	= np.sin(theta)
		x0 	= a*rho
		y0	= b*rho
		if round(np.rad2deg(a)) == 0: # Horizontal lines - search y_points
			y_m_above 	= np.argwhere(y_points >= y0 - radi_threshold).T[0]
			y_m_below 	= np.argwhere(y_points < y0 + radi_threshold).T[0]
			y_m_ab_b 	= np.in1d(y_m_above, y_m_below)
			y_m_be_b 	= np.in1d(y_m_below, y_m_above)
			y_m_ab_in 	= np.argwhere(y_m_ab_b == True).T[0]
			y_m_be_in 	= np.argwhere(y_m_be_b == True).T[0]
			y_m_temp	= np.zeros(len(y_m_ab_in) + len(y_m_be_in))
			len_ab 		= len(y_m_ab_in)
			for i in range(len(y_m_temp)):
				if i < len_ab:
					y_m_temp[i] = y_m_above[y_m_ab_in[i]]
				else:
					y_m_temp[i] = y_m_below[y_m_be_in[i-len_ab]]
			y_m = np.unique(y_m_temp)
			if len(y_m) < 2: # Ignore lines of only a single point.
				continue
			y_dist = {}
			for y in y_m:
				y = int(y)
				y_dist[x_points[y]] = y_points[y]
			sorted_y = sorted(y_dist.items(), key=operator.itemgetter(0))
			min_point = sorted_y[0]
			max_point = sorted_y[-1:][0]

			xp = np.zeros(len(y_dist))
			fp = np.zeros(len(y_dist))
			i = 0
			for x, y in sorted_y:
				xp[i] = x 
				fp[i] = y
				i += 1
			y_inter = np.interp([min_point[0], max_point[0]], xp, fp)
			x1 = int(round(min_point[0]))
			y1 = int(round(y_inter[0]))
			x2 = int(round(max_point[0]))
			y2 = int(round(y_inter[1]))

			min_hor_points[0].append(min_point[0]) #x
			min_hor_points[1].append(y_inter[0]) #y
			max_hor_points[0].append(max_point[0]) #x
			max_hor_points[1].append(y_inter[1]) #y
			average_line_x_pos = y_inter[0] + (y_inter[1] - y_inter[0])/2
			if average_line_x_pos > max_hor_line_x_pos:
				max_hor_line_x_pos 	= average_line_x_pos
				max_hor_line = ((x1,y1), (x2,y2))
			if average_line_x_pos < min_hor_line_x_pos:
				min_hor_line_x_pos 	= average_line_x_pos
				min_hor_line = ((x1,y1), (x2,y2))
			bounded_lines[0].append(((x1,y1), (x2,y2)))
		elif round(np.rad2deg(b)) == 0: # vertical lines - search x_points
			x_m_above 	= np.argwhere(x_points >= x0 - radi_threshold).T[0]
			x_m_below 	= np.argwhere(x_points < x0 + radi_threshold).T[0]
			x_m_ab_b 	= np.in1d(x_m_above, x_m_below)
			x_m_be_b 	= np.in1d(x_m_below, x_m_above)
			x_m_ab_in 	= np.argwhere(x_m_ab_b == True).T[0]
			x_m_be_in 	= np.argwhere(x_m_be_b == True).T[0]
			x_m_temp	= np.zeros(len(x_m_ab_in) + len(x_m_be_in))
			len_ab 		= len(x_m_ab_in)
			for i in range(len(x_m_temp)):
				if i < len_ab:
					x_m_temp[i] = x_m_above[x_m_ab_in[i]]
				else:
					x_m_temp[i] = x_m_below[x_m_be_in[i-len_ab]]
			x_m = np.unique(x_m_temp)
			if len(x_m) < 2: # Ignore lines of only a single point.
				continue
			x_dist = {}
			for x in x_m:
				x = int(x)
				x_dist[y_points[x]] = x_points[x]
			sorted_x = sorted(x_dist.items(), key=operator.itemgetter(0))
			min_point = sorted_x[0]
			max_point = sorted_x[-1:][0]

			xp = np.zeros(len(x_dist))
			fp = np.zeros(len(x_dist))
			i = 0
			for x, y in sorted_x:
				xp[i] = x 
				fp[i] = y
				i += 1
			x_inter = np.interp([min_point[1], max_point[1]], xp, fp)
			x1 = int(round(x_inter[0]))
			y1 = int(round(min_point[0]))
			x2 = int(round(x_inter[1]))
			y2 = int(round(max_point[0]))

			min_vert_points[0].append(x_inter[0]) #x
			min_vert_points[1].append(min_point[0]) #y
			max_vert_points[0].append(x_inter[1]) #x
			max_vert_points[1].append(max_point[0]) #y
			average_line_y_pos = x_inter[0] + (x_inter[1] - x_inter[0])/2
			if average_line_y_pos > max_vert_line_y_pos:
				max_vert_line_y_pos  = average_line_y_pos
				max_vert_line = ((x1,y1), (x2,y2))
			if average_line_y_pos < min_vert_line_y_pos:
				min_vert_line_y_pos  = average_line_y_pos
				min_vert_line = ((x1,y1), (x2,y2))
			bounded_lines[1].append(((x1,y1), (x2,y2)))
		else:
			raise DroneVisionError('find_line_limits_unexpected_angle')

		if draw_bounded_lines:
			color 		= (0,255,255)
			line_thick 	= 2
			if draw_arrowed_bounded_lines:
				tipLength = 0.05
				cv2.arrowedLine(frame, (x1,y1), (x2,y2), color, line_thick, tipLength=tipLength)
				cv2.arrowedLine(frame, (x2,y2), (x1,y1), color, line_thick, tipLength=tipLength)
			else:
				cv2.line(frame, (x1,y1), (x2,y2), color, line_thick)

	try:
		max_min_lines = [max_hor_line, min_hor_line, max_vert_line, min_vert_line]
	except:
		raise DroneVisionError('find_line_limits_no_hor_or_vert_found')

	if draw_max_min_lines and len(hough_lines) > 0:
		cv2.line(frame, max_hor_line[0], max_hor_line[1], (255,0,255),4)
		cv2.line(frame, min_hor_line[0], min_hor_line[1], (255,0,255),4)
		cv2.line(frame, max_vert_line[0], max_vert_line[1], (255,0,255),4)
		cv2.line(frame, min_vert_line[0], min_vert_line[1], (255,0,255),4)

	return frame, bounded_lines, max_min_lines

def DrawHoughLine(frame, hough_line, color):
	'''
	 @brief Draw hough line on frame.
	 	Hough lines are give in rho, theta coordinates.
	
	 @param frame
	 @param hough_line (rho, theta)
	 @param color RGB color (R,G,B)

	 @return frame (with drawn line)
	'''
	frame = CheckColor(frame)
	width, height = GetShape(frame)
	size = math.sqrt(math.pow(width, 2.0) + math.pow(height, 2.0))
	rho, theta = hough_line
	if not(np.isnan(theta)) and not(np.isnan(rho)):
		a 	= np.cos(theta)
		b 	= np.sin(theta)
		x0 	= a*rho
		y0	= b*rho
		x1 	= int(x0 + size*(-b))
		y1 	= int(y0 + size*(a))
		x2 	= int(x0 - size*(-b))
		y2 	= int(y0 - size*(a))
		cv2.line(frame, (x1,y1), (x2,y2), color, 3)
	return frame

def DrawHoughLines(frame, hough_lines, color=(255,0,0)):
	'''
	 @brief Draw hough lines on frame.
	 	Hough lines are give in rho, theta coordinates.
	
	 @param frame
	 @param hough_lines List of hough lines (rho, theta)
	 @param color RGB color (R,G,B)

	 @return frame (with drawn lines)
	'''
	for hough_line in hough_lines:
		frame = DrawHoughLine(frame, hough_line, color)
	return frame

def PrintHoughAccumulator(accumulator, id_m, rhos):
	'''
	 @brief Print Hough transform accumulator in matplotlib figure.

	 @param accumulator
	'''
	acc_shape 	= GetShape(accumulator)
	ylabs 		= np.deg2rad(np.arange(-0.0, 180.0, 1))
	acc_map 	= np.zeros((acc_shape[0], len(ylabs)), dtype=np.uint16)
	id_acc 		= np.argwhere(accumulator > 0)
	for i in range(len(id_acc)):

		idx 		= id_acc[i][0]*acc_shape[1] + id_acc[i][1]
		rho 		= rhos[idx / acc_shape[1]]
		acc_map[int(rho), idx % acc_shape[1]] += 1

	acc_map = cv2.cvtColor(acc_map, cv2.COLOR_GRAY2BGR)

	for i in range(len(id_m)):
		idx 		= id_m[i][0]*acc_shape[1] + id_m[i][1]
		rho 		= rhos[idx / acc_shape[1]]
		cv2.circle(acc_map,(int(rho), idx % acc_shape[1]), 1, (0,0,255), -1)

	acc_map = CheckGrayScale(acc_map)

	touple_frame_plot = []
	touple_frame_plot.append(('Hough line accumulator', acc_map.T))
	MatplotShow(touple_frame_plot, 'Hough_line_accumulator')

def concatenateLines(lines, change_rho):
	'''
	 @brief Concatene hough lines which are within a distance.

	 @returns concatenated lines. (sorted from min -> max theta)
	'''
	theta_dict = {}
	for rho, theta in lines:
		if not(theta in theta_dict):
			theta_dict[theta] = []
		theta_dict[theta].append(rho)
	con_lines = []
	for theta in theta_dict:
		theta_dict[theta].sort()
		j = 0
		for i in range(1, len(theta_dict[theta])):
			if abs(theta_dict[theta][i] - theta_dict[theta][i-1]) > change_rho:
				con_lines.append((np.median(theta_dict[theta][j:i]), theta))
				j = i
		con_lines.append((np.median(theta_dict[theta][j:]), theta))
	return con_lines

def HoughLineEdgePoints(frame, edge_points, horizontal_points, hough_peak_param=1.2, dilation_kernel_size=3, dilation_iterations=1):
	'''
	 @brief Speeded up Houg lines transform for lines by iterating over known key point positions.
	 	Inspired by: https://alyssaq.github.io/2014/understanding-hough-transform/

	 @param frame
	 @param edge_points List of detected edge points as [[x], [y]]
	 @param horizontal_points Set True for detecting horizontal lines, and False for detecting vertical lines.
	 		The lines will be focused around the vertical or horizontal axis.
	 @param hough_peak_param Delimiter for increasing number of peaks to validate for finding the most significant peaks. Must be >= 1.0.
	 @param dilation_kernel_size (Increase strong areas of peak points by dilation. 
	 	Set the kernel size for dilation as a dilation_kernel_size*dilation_kernel_size (f.eks 3*3 kernel size). 
	 	Set to 1 to give the dilation no effect. (default=3))
	 @param dilation_iterations (Number of dilation iterations to increase the width of strong areas (default=1))

	 @return (rho, theta) (Distance and angle of the most significant edge.)
	'''


	# Rho and Theta ranges
	if horizontal_points:
		start_degree 	= 0.0
		end_degree 		= 45.0
		step_degree 	= 1.0
		thetas_small	= np.deg2rad(np.arange(start_degree, end_degree, step_degree))
		start_degree 	= 135.0
		end_degree 		= 180.0
		step_degree 	= 1.0
		thetas_big		= np.deg2rad(np.arange(start_degree, end_degree, step_degree))
		thetas 			= np.concatenate((thetas_small, thetas_big))
	else: # vertical points
		start_degree 	= 45.0
		end_degree 		= 135.0
		step_degree 	= 1.0
		thetas			= np.deg2rad(np.arange(start_degree, end_degree, step_degree))

	width, height 	= GetShape(frame)
	diag_len 		= np.ceil(np.sqrt(width * width + height * height))   # max_dist
	rhos 			= np.linspace(-np.int(diag_len), np.int(diag_len), np.int(diag_len) * 2)

	# Cache some reusable values
	cos_t 		= np.cos(thetas)
	sin_t 		= np.sin(thetas)
	num_thetas 	= len(thetas)

	# Hough accumulator array of theta vs rho
	accumulator = np.zeros((int(2 * diag_len), num_thetas), dtype=np.float32)
	acc_shape 	= GetShape(accumulator)

	# Vote in the hough accumulator
	for i in range(len(edge_points[0])):
		x = edge_points[0][i]
		y = edge_points[1][i]

		for t_idx in range(num_thetas):
			# Calculate rho. diag_len is added for a positive index
			rho = int(round(x * cos_t[t_idx] + y * sin_t[t_idx]) + diag_len)
			accumulator[rho, t_idx] += 1

	# Dilate the accumulator so that close neighboring voting points are stronger, and add the Gaussian smoothed accumulator to highlight the strongest peak in the strongest areas.
	accumulator = cv2.dilate(accumulator, np.ones((dilation_kernel_size,dilation_kernel_size), dtype=np.uint8), iterations=dilation_iterations)
	accumulator += cv2.GaussianBlur(accumulator, (dilation_kernel_size,dilation_kernel_size), 0) # Let sigma be calculated according to the kernel size. See cv2 doc.

	# Peak finding based on max votes.
	id_m 		= np.argwhere(accumulator >= np.max(accumulator)/hough_peak_param)
	len_id_m 	= len(id_m)
	rhos_res 	= [0]*len_id_m
	thetas_res 	= [0]*len_id_m
	for i in range(len_id_m):
		idx 			= id_m[i][0]*acc_shape[1] + id_m[i][1]
		rhos_res[i] 	= rhos[idx / acc_shape[1]]
		thetas_res[i] 	= thetas[idx % acc_shape[1]]

	rho 	= np.median(rhos_res)
	theta 	= np.median(thetas_res)

	return (rho, theta)

def HoughLinesPointMatrix(frame, keypoints, min_lines=2, radi_threshold=None, radi_threshold_tuning_param=2.0):
	'''
	 @brief Speeded up Houg lines transform for lines by iterating over known key point positions.
	 	Inspired by: https://alyssaq.github.io/2014/understanding-hough-transform/

	 @param frame
	 @param keypoints List of detected points (using the blob detection algorithm.)
	 @param min_lines Minimum lines to finally end up with. Set to -1 to hinder any concateniation of detected lines.
	 @param radi_threshold Threshold in pixels to search for in vertical and horizontal axis (If none, then it is set to the biggest blob size)
	 @param radi_threshold_tuning_param Threshold tuning parameter (default=2 when using biggest blob size. < 0.5 is recommended when using distance betwen blobs).

	 @return hough_lines Returned as list of touples (rho, theta)
	'''
	# Rho and Theta ranges
	thetas 			= np.deg2rad(np.array([0.0, 90.0]))
	width, height 	= GetShape(frame)
	diag_len 		= np.ceil(np.sqrt(width * width + height * height))   # max_dist
	rhos 			= np.linspace(-np.int(diag_len), np.int(diag_len), np.int(diag_len) * 2)

	# Cache some resuable values
	cos_t 		= np.cos(thetas)
	sin_t 		= np.sin(thetas)
	num_thetas 	= len(thetas)

	# Hough accumulator array of theta vs rho
	accumulator = np.zeros((int(2 * diag_len), num_thetas), dtype=np.float32)
	acc_shape 	= GetShape(accumulator)

	biggest_point = 0
	# Vote in the hough accumulator
	for i in range(len(keypoints)):
		x = keypoints[i].pt[0]
		y = keypoints[i].pt[1]

		if keypoints[i].size > biggest_point:
			biggest_point = keypoints[i].size

		for t_idx in range(num_thetas):
			# Calculate rho. diag_len is added for a positive index
			rho = int(round(x * cos_t[t_idx] + y * sin_t[t_idx]) + diag_len)
			accumulator[rho, t_idx] += 1

	# Peak finding based on max votes
	id_m 		= np.argwhere(accumulator >= 1)
	len_id_m 	= len(id_m)
	hough_lines = [0]*len_id_m
	#PrintHoughAccumulator(accumulator, id_m, rhos)
	for i in range(len_id_m):
		idx 			= id_m[i][0]*acc_shape[1] + id_m[i][1]
		rho 			= rhos[idx / acc_shape[1]]
		theta 			= thetas[idx % acc_shape[1]]
		hough_lines[i] 	= (rho, theta)

	if radi_threshold != None:
		origin_threshold = radi_threshold*radi_threshold_tuning_param
	else:
		origin_threshold = biggest_point
	threshold = origin_threshold
		
	n_lines = len(hough_lines) + 1 #Stop concatenating when there is no change in n_lines
	while len(hough_lines) > min_lines and n_lines - len(hough_lines) > 0 and min_lines > 0:
		n_lines 	= len(hough_lines)
		hough_lines = concatenateLines(hough_lines, threshold)
		threshold 	+= origin_threshold/2
	return hough_lines # returned as touples of (rho, theta)